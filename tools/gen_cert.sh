#!/bin/sh
# SPDX-License-Identifier: MIT
# Generates the board's HTTPS identity: a one-off local root CA (P-256,
# name-constrained to the board's own networks) and a server certificate for
# 192.168.4.1 (softAP) + iPhone-hotspot addresses. The CA private key is
# deleted right after signing, so nobody (not even this repo) can mint more
# certificates under a CA a phone may have trusted. Re-running creates a NEW
# CA: phones that installed the old one must install the new ca.crt again.
set -eu
OUT="$(cd "$(dirname "$0")/.." && pwd)/components/app_link/certs"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
mkdir -p "$OUT"
cd "$TMP"
cat > ca.cnf <<'CNF'
[req]
distinguished_name = dn
prompt = no
x509_extensions = v3_ca
[dn]
CN = FlightPFD Local CA
O = flight_pfd (this board only)
[v3_ca]
basicConstraints = critical, CA:TRUE, pathlen:0
keyUsage = critical, keyCertSign, cRLSign
subjectKeyIdentifier = hash
nameConstraints = critical, permitted;IP:192.168.4.0/255.255.255.0, permitted;IP:172.20.10.0/255.255.255.240, permitted;DNS:.local
CNF
openssl ecparam -name prime256v1 -genkey -noout -out ca.key
openssl req -x509 -new -key ca.key -sha256 -days 3650 -config ca.cnf -out ca.pem
SAN="IP:192.168.4.1"
i=2; while [ $i -le 14 ]; do SAN="$SAN,IP:172.20.10.$i"; i=$((i+1)); done
SAN="$SAN,DNS:feixingbiao.local"
cat > leaf.cnf <<CNF
[req]
distinguished_name = dn
prompt = no
[dn]
CN = feixingbiao.local
O = flight_pfd
[v3]
basicConstraints = critical, CA:FALSE
keyUsage = critical, digitalSignature
extendedKeyUsage = serverAuth
subjectAltName = $SAN
authorityKeyIdentifier = keyid
subjectKeyIdentifier = hash
CNF
openssl ecparam -name prime256v1 -genkey -noout -out leaf.key
openssl req -new -key leaf.key -config leaf.cnf -out leaf.csr
# Apple requires <= 825 days for TLS server certificates.
openssl x509 -req -in leaf.csr -CA ca.pem -CAkey ca.key -CAcreateserial -sha256 -days 820 \
    -extfile leaf.cnf -extensions v3 -out leaf.pem
openssl verify -CAfile ca.pem leaf.pem
shred -u ca.key 2>/dev/null || rm -f ca.key
cat leaf.pem ca.pem > "$OUT/server_chain.pem"
openssl pkey -in leaf.key -out "$OUT/server_key.pem"
openssl x509 -in ca.pem -outform der -out "$OUT/ca.crt"
cp ca.pem "$OUT/ca.pem"
echo "wrote $OUT (CA key destroyed)"
