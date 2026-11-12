# HTTPS identity for the phone page. Private keys are never committed: on the
# first configure, tools/gen_cert.sh creates a fresh local CA + server cert in
# certs/ (git-ignored). Each build owner therefore has their own CA.
set(PFD_CERT_DIR "${CMAKE_CURRENT_LIST_DIR}/certs")
if(NOT CMAKE_BUILD_EARLY_EXPANSION AND NOT EXISTS "${PFD_CERT_DIR}/server_key.pem")
    message(STATUS "app_link: generating HTTPS certificate (tools/gen_cert.sh)")
    execute_process(COMMAND sh "${CMAKE_CURRENT_LIST_DIR}/../../tools/gen_cert.sh"
                    RESULT_VARIABLE pfd_cert_rc)
    if(NOT pfd_cert_rc EQUAL 0)
        message(FATAL_ERROR "app_link: tools/gen_cert.sh failed (needs openssl)")
    endif()
endif()

idf_component_register(SRCS "link_core.c" "link_esp.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_wifi esp_netif esp_event esp_http_server esp_https_server nvs_flash esp_timer
                  esp_hw_support freertos log heap lwip
    EMBED_TXTFILES "page/index.html" "certs/server_chain.pem" "certs/server_key.pem"
    EMBED_FILES "certs/ca.crt")
