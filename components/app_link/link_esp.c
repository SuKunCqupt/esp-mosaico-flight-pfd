// SPDX-License-Identifier: MIT
/* Phone GPS link, device side.
 *  - Wi-Fi softAP "飞行表-XXXX" (WPA2, 8-digit password kept in NVS), optional
 *    station uplink (phone hotspot / home Wi-Fi) configured from the page.
 *  - HTTPS server on :443 (self-made CA + P-256 server cert, embedded) serving
 *    the phone page and POST /api/gps; plain HTTP :80 only redirects to HTTPS.
 *  - No DNS server on purpose: phones see "no internet" instead of popping a
 *    captive-portal mini browser, which cannot use geolocation.
 * Wi-Fi, lwIP and both servers run on core 1 at low priority; the frame loop
 * on core 0 only takes a short spinlock in pfd_link_read(). */
#include "include/pfd_gps.h"
#include <stdio.h>
#include <string.h>
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_https_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "pfd_link";

extern const char page_start[] asm("_binary_index_html_start");
extern const char page_end[] asm("_binary_index_html_end");
extern const uint8_t chain_start[] asm("_binary_server_chain_pem_start");
extern const uint8_t chain_end[] asm("_binary_server_chain_pem_end");
extern const uint8_t key_start[] asm("_binary_server_key_pem_start");
extern const uint8_t key_end[] asm("_binary_server_key_pem_end");
extern const uint8_t ca_start[] asm("_binary_ca_crt_start");
extern const uint8_t ca_end[] asm("_binary_ca_crt_end");

#define NVS_NS "pfd"
#define STA_MAX_ATTEMPTS 3

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static pfd_gps_track_t s_track;
static pfd_link_snapshot_t s_net; /* network fields only */
static esp_netif_t *s_sta_netif;
static httpd_handle_t s_https, s_http;
/* station control (guarded by s_mux) */
static bool s_sta_wanted, s_sta_reconfig, s_sta_shared;
static char s_sta_pass[65];
static int s_sta_attempts;
static int64_t s_sta_next_us;
static uint32_t s_last_log_packets;

static double now_s(void) { return esp_timer_get_time() / 1e6; }

void pfd_link_read(pfd_link_snapshot_t *out)
{
    double t = now_s();
    taskENTER_CRITICAL(&s_mux);
    *out = s_net;
    pfd_gps_fill(&s_track, t, out);
    taskEXIT_CRITICAL(&s_mux);
}

/* ---------------- NVS ---------------- */

static void load_or_make_ap_pass(char *pass, size_t cap)
{
    nvs_handle_t h;
    size_t len = cap;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        snprintf(pass, cap, "%08lu", (unsigned long)(esp_random() % 100000000UL));
        return;
    }
    if (nvs_get_str(h, "ap_pass", pass, &len) != ESP_OK || strlen(pass) != 8) {
        /* 8 digits: easy to type, never 0-prefixed-ambiguous */
        snprintf(pass, cap, "%08lu", (unsigned long)(10000000UL + esp_random() % 90000000UL));
        if (nvs_set_str(h, "ap_pass", pass) == ESP_OK) (void)nvs_commit(h);
    }
    nvs_close(h);
}

/* Station credentials: own ("sta_ssid"/"sta_pass") first; otherwise the Wi-Fi
 * the device already knows from Vibe Mode (read-only, never logged). */
static bool load_sta(char *ssid, size_t scap, char *pass, size_t pcap, bool *shared)
{
    nvs_handle_t h;
    *shared = false;
    uint8_t no_shared = 0;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        size_t sl = scap, pl = pcap;
        bool ok = nvs_get_str(h, "sta_ssid", ssid, &sl) == ESP_OK && nvs_get_str(h, "sta_pass", pass, &pl) == ESP_OK;
        (void)nvs_get_u8(h, "sta_off", &no_shared);
        nvs_close(h);
        if (ok && ssid[0]) return true;
    }
    if (no_shared) return false;
    esp_err_t err = nvs_open_from_partition("sysmeta", "shared", NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_INITIALIZED || err == ESP_ERR_NVS_PART_NOT_FOUND) {
        if (nvs_flash_init_partition("sysmeta") == ESP_OK)
            err = nvs_open_from_partition("sysmeta", "shared", NVS_READONLY, &h);
    }
    if (err != ESP_OK) return false;
    size_t sl = scap, pl = pcap;
    bool ok = nvs_get_str(h, "wifi_ssid", ssid, &sl) == ESP_OK && nvs_get_str(h, "wifi_password", pass, &pl) == ESP_OK;
    nvs_close(h);
    *shared = ok && ssid[0];
    return *shared;
}

static esp_err_t save_sta(const char *ssid, const char *pass, bool forget)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    if (forget) {
        (void)nvs_erase_key(h, "sta_ssid");
        (void)nvs_erase_key(h, "sta_pass");
        err = nvs_set_u8(h, "sta_off", 1);
    } else {
        err = nvs_set_str(h, "sta_ssid", ssid);
        if (err == ESP_OK) err = nvs_set_str(h, "sta_pass", pass);
        if (err == ESP_OK) err = nvs_set_u8(h, "sta_off", 0);
    }
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

/* ---------------- Wi-Fi events ---------------- */

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        taskENTER_CRITICAL(&s_mux);
        s_net.ap_clients++;
        taskEXIT_CRITICAL(&s_mux);
        ESP_LOGI(TAG, "phone joined hotspot (clients=%d)", s_net.ap_clients);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        taskENTER_CRITICAL(&s_mux);
        if (s_net.ap_clients > 0) s_net.ap_clients--;
        taskEXIT_CRITICAL(&s_mux);
        ESP_LOGI(TAG, "phone left hotspot (clients=%d)", s_net.ap_clients);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *d = data;
        taskENTER_CRITICAL(&s_mux);
        bool was = s_net.sta == PFD_STA_CONNECTED;
        s_net.sta_url[0] = 0;
        if (s_sta_wanted) {
            s_net.sta = s_sta_attempts >= STA_MAX_ATTEMPTS ? PFD_STA_FAILED : PFD_STA_CONNECTING;
            /* lost an established link: start a fresh retry series */
            if (was) s_sta_attempts = 0;
            s_sta_next_us = esp_timer_get_time() + (was ? 2000000 : 10000000LL * (1 << s_sta_attempts));
        } else {
            s_net.sta = PFD_STA_OFF;
        }
        taskEXIT_CRITICAL(&s_mux);
        ESP_LOGI(TAG, "station disconnected (reason %d)", d ? d->reason : -1);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *e = data;
        taskENTER_CRITICAL(&s_mux);
        s_net.sta = PFD_STA_CONNECTED;
        s_sta_attempts = 0;
        snprintf(s_net.sta_url, sizeof(s_net.sta_url), "https://" IPSTR, IP2STR(&e->ip_info.ip));
        taskEXIT_CRITICAL(&s_mux);
        ESP_LOGI(TAG, "station connected: %s", s_net.sta_url);
    } else if (base == IP_EVENT && id == IP_EVENT_ASSIGNED_IP_TO_CLIENT) {
        ESP_LOGI(TAG, "hotspot gave a phone its address");
    }
}

/* Called from the link task only. */
static void sta_apply(void)
{
    char ssid[33] = {0}, pass[65] = {0};
    bool shared = false;
    bool have = load_sta(ssid, sizeof(ssid), pass, sizeof(pass), &shared);
    (void)esp_wifi_disconnect();
    taskENTER_CRITICAL(&s_mux);
    s_sta_wanted = have;
    s_sta_shared = shared;
    s_sta_attempts = 0;
    s_sta_next_us = esp_timer_get_time() + 500000;
    s_net.sta = have ? PFD_STA_CONNECTING : PFD_STA_OFF;
    snprintf(s_net.sta_ssid, sizeof(s_net.sta_ssid), "%s", have ? ssid : "");
    s_net.sta_url[0] = 0;
    memcpy(s_sta_pass, pass, sizeof(s_sta_pass));
    taskEXIT_CRITICAL(&s_mux);
    (void)esp_wifi_set_mode(have ? WIFI_MODE_APSTA : WIFI_MODE_AP);
    if (have) {
        wifi_config_t sc = {0};
        memcpy(sc.sta.ssid, ssid, strnlen(ssid, sizeof(sc.sta.ssid)));
        memcpy(sc.sta.password, pass, strnlen(pass, sizeof(sc.sta.password)));
        sc.sta.threshold.authmode = pass[0] ? WIFI_AUTH_WPA_PSK : WIFI_AUTH_OPEN;
        sc.sta.pmf_cfg.capable = true;
        (void)esp_wifi_set_config(WIFI_IF_STA, &sc);
        ESP_LOGI(TAG, "station configured (%s credentials)", shared ? "device Wi-Fi" : "page");
    }
    memset(pass, 0, sizeof(pass));
}

/* ---------------- HTTP handlers ---------------- */

static esp_err_t send_json(httpd_req_t *req, const char *json)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, json);
}

static int read_body(httpd_req_t *req, char *buf, size_t cap)
{
    if (req->content_len >= cap) return -1;
    size_t got = 0;
    while (got < req->content_len) {
        int r = httpd_req_recv(req, buf + got, req->content_len - got);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (r <= 0) return -1;
        got += (size_t)r;
    }
    buf[got] = 0;
    return (int)got;
}

static const char *sta_name(pfd_sta_state_t s)
{
    switch (s) {
    case PFD_STA_CONNECTING: return "connecting";
    case PFD_STA_CONNECTED: return "connected";
    case PFD_STA_FAILED: return "failed";
    default: return "off";
    }
}

/* minimal JSON string escaping for SSIDs */
static void json_esc(char *o, size_t cap, const char *s)
{
    size_t n = 0;
    for (; *s && n + 7 < cap; ++s) {
        unsigned char c = (unsigned char)*s;
        if (c == '"' || c == '\\') { o[n++] = '\\'; o[n++] = (char)c; }
        else if (c < 0x20) n += (size_t)snprintf(o + n, cap - n, "\\u%04x", c);
        else o[n++] = (char)c;
    }
    o[n] = 0;
}

static void info_json(char *buf, size_t cap, uint32_t packets)
{
    pfd_link_snapshot_t n;
    taskENTER_CRITICAL(&s_mux);
    n = s_net;
    taskEXIT_CRITICAL(&s_mux);
    char ssid[80], ap[96];
    json_esc(ssid, sizeof(ssid), n.sta_ssid);
    json_esc(ap, sizeof(ap), n.ap_ssid);
    const char *ip = n.sta_url[0] ? n.sta_url + 8 : "";
    snprintf(buf, cap, "{\"ok\":true,\"n\":%lu,\"ap\":\"%s\",\"sta\":\"%s\",\"sta_ssid\":\"%s\",\"sta_ip\":\"%s\"}",
             (unsigned long)packets, ap, sta_name(n.sta), ssid, ip);
}

static esp_err_t h_page(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, page_start, page_end - page_start - 1); /* EMBED_TXTFILES adds a NUL */
}

static esp_err_t h_ca(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/x-x509-ca-cert");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"FlightPFD-CA.crt\"");
    return httpd_resp_send(req, (const char *)ca_start, ca_end - ca_start);
}

static esp_err_t h_info(httpd_req_t *req)
{
    char out[400];
    uint32_t packets;
    taskENTER_CRITICAL(&s_mux);
    packets = s_track.packets;
    taskEXIT_CRITICAL(&s_mux);
    info_json(out, sizeof(out), packets);
    return send_json(req, out);
}

static esp_err_t h_gps(httpd_req_t *req)
{
    char body[512];
    if (read_body(req, body, sizeof(body)) < 0) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
    pfd_gps_msg_t m;
    if (!pfd_gps_parse(body, strlen(body), &m)) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad json");
    bool first, first_fix;
    uint32_t packets;
    taskENTER_CRITICAL(&s_mux);
    first = s_track.packets == 0;
    first_fix = m.has_pos && !s_track.has_fix;
    pfd_gps_apply(&s_track, &m, now_s());
    packets = s_track.packets;
    taskEXIT_CRITICAL(&s_mux);
    if (first) ESP_LOGI(TAG, "phone page connected, first packet");
    if (first_fix)
        ESP_LOGI(TAG, "first GPS fix: acc=%.0fm alt=%s spd=%s", m.acc, m.alt_valid ? "yes" : "null",
                 m.spd_valid ? "yes" : "null");
    char out[400];
    info_json(out, sizeof(out), packets);
    return send_json(req, out);
}

static esp_err_t h_wifi(httpd_req_t *req)
{
    char body[256], ssid[33] = {0}, pass[65] = {0};
    bool forget = false;
    if (read_body(req, body, sizeof(body)) < 0) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
    size_t len = strlen(body);
    (void)pfd_json_bool(body, len, "forget", &forget);
    if (!forget) {
        if (!pfd_json_string(body, len, "ssid", ssid, sizeof(ssid)) || !ssid[0])
            return send_json(req, "{\"ok\":false,\"msg\":\"请填写 Wi-Fi 名称\"}");
        (void)pfd_json_string(body, len, "pass", pass, sizeof(pass));
        if (pass[0] && strlen(pass) < 8) return send_json(req, "{\"ok\":false,\"msg\":\"密码至少 8 位\"}");
    }
    memset(body, 0, sizeof(body));
    esp_err_t err = save_sta(ssid, pass, forget);
    memset(pass, 0, sizeof(pass));
    if (err != ESP_OK) return send_json(req, "{\"ok\":false,\"msg\":\"保存失败\"}");
    taskENTER_CRITICAL(&s_mux);
    s_sta_reconfig = true;
    taskEXIT_CRITICAL(&s_mux);
    ESP_LOGI(TAG, "station settings %s from page", forget ? "cleared" : "saved");
    return send_json(req, forget ? "{\"ok\":true,\"msg\":\"已取消连接，只用飞行表热点\"}"
                                 : "{\"ok\":true,\"msg\":\"已保存，正在连接…连上后飞行表屏幕会显示新网址\"}");
}

static esp_err_t h_nocontent(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t h_other(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t h_redirect(httpd_req_t *req)
{
    /* Send the phone to the same address it used, over HTTPS. */
    char host[64] = {0}, loc[96] = "https://192.168.4.1/";
    if (httpd_req_get_hdr_value_str(req, "Host", host, sizeof(host)) == ESP_OK && host[0]) {
        size_t n = 0;
        while (host[n] && ((host[n] >= '0' && host[n] <= '9') || (host[n] >= 'a' && host[n] <= 'z') ||
                           (host[n] >= 'A' && host[n] <= 'Z') || host[n] == '.' || host[n] == '-'))
            ++n;
        if (n && (host[n] == 0 || host[n] == ':')) {
            host[n] = 0;
            snprintf(loc, sizeof(loc), "https://%s/", host);
        }
    }
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", loc);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t start_servers(void)
{
    httpd_ssl_config_t c = HTTPD_SSL_CONFIG_DEFAULT();
    c.servercert = chain_start;
    c.servercert_len = chain_end - chain_start;
    c.prvtkey_pem = key_start;
    c.prvtkey_len = key_end - key_start;
    c.httpd.core_id = 1;
    c.httpd.task_priority = 5;
    c.httpd.stack_size = 10240;
    c.httpd.max_open_sockets = 4;
    c.httpd.max_uri_handlers = 10;
    c.httpd.lru_purge_enable = true;
    c.httpd.uri_match_fn = httpd_uri_match_wildcard;
    esp_err_t err = httpd_ssl_start(&s_https, &c);
    if (err != ESP_OK) return err;
    const httpd_uri_t uris[] = {
        {.uri = "/", .method = HTTP_GET, .handler = h_page},
        {.uri = "/index.html", .method = HTTP_GET, .handler = h_page},
        {.uri = "/ca.crt", .method = HTTP_GET, .handler = h_ca},
        {.uri = "/api/info", .method = HTTP_GET, .handler = h_info},
        {.uri = "/api/gps", .method = HTTP_POST, .handler = h_gps},
        {.uri = "/api/wifi", .method = HTTP_POST, .handler = h_wifi},
        {.uri = "/favicon.ico", .method = HTTP_GET, .handler = h_nocontent},
        {.uri = "/*", .method = HTTP_GET, .handler = h_other},
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); ++i) httpd_register_uri_handler(s_https, &uris[i]);

    httpd_config_t p = HTTPD_DEFAULT_CONFIG();
    p.core_id = 1;
    p.task_priority = 4;
    p.stack_size = 4096;
    p.max_open_sockets = 3;
    p.ctrl_port = 32770;
    p.lru_purge_enable = true;
    p.uri_match_fn = httpd_uri_match_wildcard;
    if (httpd_start(&s_http, &p) == ESP_OK) {
        const httpd_uri_t r = {.uri = "/*", .method = HTTP_GET, .handler = h_redirect};
        const httpd_uri_t ca = {.uri = "/ca.crt", .method = HTTP_GET, .handler = h_ca};
        httpd_register_uri_handler(s_http, &ca);
        httpd_register_uri_handler(s_http, &r);
    }
    return ESP_OK;
}

/* ---------------- bring-up task ---------------- */

static esp_err_t wifi_up(void)
{
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK) return err;
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    esp_netif_t *ap = esp_netif_create_default_wifi_ap();
    s_sta_netif = esp_netif_create_default_wifi_sta();
    if (!ap || !s_sta_netif) return ESP_FAIL;
    wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
    if ((err = esp_wifi_init(&ic)) != ESP_OK) return err;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, on_event, NULL, NULL);
    (void)esp_wifi_set_storage(WIFI_STORAGE_RAM);

    uint8_t mac[6] = {0};
    (void)esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    char pass[16];
    load_or_make_ap_pass(pass, sizeof(pass));
    wifi_config_t ac = {0};
    int sl = snprintf((char *)ac.ap.ssid, sizeof(ac.ap.ssid), "飞行表-%02X%02X", mac[4], mac[5]);
    ac.ap.ssid_len = (uint8_t)sl;
    snprintf((char *)ac.ap.password, sizeof(ac.ap.password), "%s", pass);
    ac.ap.channel = 6;
    ac.ap.max_connection = 4;
    ac.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ac.ap.pmf_cfg.required = false;
    if ((err = esp_wifi_set_mode(WIFI_MODE_AP)) != ESP_OK) return err;
    if ((err = esp_wifi_set_config(WIFI_IF_AP, &ac)) != ESP_OK) return err;
    if ((err = esp_wifi_start()) != ESP_OK) return err;
    (void)esp_wifi_set_ps(WIFI_PS_NONE);

    esp_netif_ip_info_t ip = {0};
    (void)esp_netif_get_ip_info(ap, &ip);
    taskENTER_CRITICAL(&s_mux);
    snprintf(s_net.ap_ssid, sizeof(s_net.ap_ssid), "%s", (const char *)ac.ap.ssid);
    snprintf(s_net.ap_pass, sizeof(s_net.ap_pass), "%s", pass);
    snprintf(s_net.ap_url, sizeof(s_net.ap_url), "https://" IPSTR, IP2STR(&ip.ip));
    s_net.wifi_up = true;
    taskEXIT_CRITICAL(&s_mux);
    ESP_LOGI(TAG, "hotspot up: ssid=%s ch=%d url=%s", s_net.ap_ssid, ac.ap.channel, s_net.ap_url);
    return ESP_OK;
}

static void link_task(void *arg)
{
    (void)arg;
    size_t heap0 = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    esp_err_t err = wifi_up();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi start failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }
    err = start_servers();
    taskENTER_CRITICAL(&s_mux);
    s_net.server_up = err == ESP_OK;
    taskEXIT_CRITICAL(&s_mux);
    if (err == ESP_OK) ESP_LOGI(TAG, "https server up on :443 (http :80 redirects)");
    else ESP_LOGE(TAG, "https server failed: %s", esp_err_to_name(err));
    sta_apply();
    ESP_LOGI(TAG, "network ready, internal heap %u -> %u bytes", (unsigned)heap0,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

    int64_t next_log = esp_timer_get_time() + 10000000;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(250));
        bool reconf, try_connect = false;
        int64_t now = esp_timer_get_time();
        taskENTER_CRITICAL(&s_mux);
        reconf = s_sta_reconfig;
        s_sta_reconfig = false;
        if (!reconf && s_sta_wanted && s_net.sta == PFD_STA_CONNECTING && s_sta_next_us && now >= s_sta_next_us) {
            if (s_sta_attempts < STA_MAX_ATTEMPTS) {
                s_sta_attempts++;
                try_connect = true;
                s_sta_next_us = 0; /* re-armed by the disconnect event */
            } else {
                s_net.sta = PFD_STA_FAILED;
            }
        }
        taskEXIT_CRITICAL(&s_mux);
        if (reconf) sta_apply();
        if (try_connect) {
            ESP_LOGI(TAG, "station connect attempt %d/%d", s_sta_attempts, STA_MAX_ATTEMPTS);
            (void)esp_wifi_connect();
        }
        if (now >= next_log) {
            pfd_link_snapshot_t s;
            pfd_link_read(&s);
            ESP_LOGI(TAG, "link phones=%d state=%d packets=%lu (+%lu) fix=%d acc=%.0fm spd=%s alt=%s sta=%s heap=%u",
                     s.ap_clients, (int)s.state, (unsigned long)s.packets,
                     (unsigned long)(s.packets - s_last_log_packets), s.has_fix, (double)s.acc_m,
                     s.speed_valid ? (s.speed_derived ? "derived" : "gps") : "none", s.alt_valid ? "gps" : "none",
                     sta_name(s.sta), (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
            s_last_log_packets = s.packets;
            next_log = now + 10000000;
        }
    }
}

int pfd_link_start(void)
{
    /* Core 1, below the IMU task (12): Wi-Fi init + reconnect bookkeeping. */
    return xTaskCreatePinnedToCore(link_task, "pfd_link", 6144, NULL, 3, NULL, 1) == pdPASS ? 0 : -1;
}
