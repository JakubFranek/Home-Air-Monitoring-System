# ESP32 SSL Certificate Problem & Solutions

## The Problem

When making HTTPS requests to APIs (like svatkyapi.cz or openweathermap.org), the ESP32 was failing with this error:

```
E (21827) esp-tls-mbedtls: mbedtls_ssl_handshake returned -0x2700
I (21827) esp-tls-mbedtls: Failed to verify peer certificate!
E (21827) esp-tls: Failed to open new connection
E (21837) transport_base: Failed to open a new connection
E (21837) HTTP_CLIENT: Connection failed, sock < 0
```

### Root Cause

SSL/TLS certificates expire and get renewed regularly (typically every 3 months to 1 year). The embedded certificate in the ESP32 firmware was outdated and didn't match what the server was presenting.

**Key insight:** You need the FULL certificate chain (server cert → intermediate CA → root CA), not just the server's certificate. The ESP32 uses this chain to verify the server's identity.

---

## Solution 1: Manual Certificate Update (What We Did First)

### Steps:

1. **Download the full certificate chain:**
   ```bash
   cd /path/to/your/project/main/
   
   # Replace domain.com with your API domain
   openssl s_client -showcerts -servername domain.com -connect domain.com:443 </dev/null 2>/dev/null | sed -n '/BEGIN CERTIFICATE/,/END CERTIFICATE/p' > cert.pem
   ```

   Example for svatkyapi.cz:
   ```bash
   openssl s_client -showcerts -servername svatkyapi.cz -connect svatkyapi.cz:443 </dev/null 2>/dev/null | sed -n '/BEGIN CERTIFICATE/,/END CERTIFICATE/p' > svatkyapicz_cert.pem
   ```

2. **Verify you got all certificates:**
   ```bash
   grep -c "BEGIN CERTIFICATE" cert.pem
   ```
   Should output 2-3 (you need the full chain)

3. **Rebuild and flash:**
   ```bash
   idf.py build flash monitor
   ```

### Pros:
- Full control over which certificates are trusted
- Smaller firmware size (only includes needed certificates)

### Cons:
- **Must manually update every 3-12 months when certificates expire**
- Requires rebuilding and reflashing firmware
- Easy to forget until it breaks

---

## Solution 2: ESP-IDF Certificate Bundle (RECOMMENDED - What We Chose)

Use ESP-IDF's built-in bundle of root CA certificates that rarely change.

### Code Changes:

**In your API code (e.g., svatky_api.c):**

Add include:
```c
#include "esp_crt_bundle.h"
```

Change the HTTP client config from:
```c
esp_http_client_config_t config = {
    .url = url_buffer,
    .event_handler = _http_event_handler,
    .cert_pem = svatkyapicz_cert_pem_start,  // OLD
};
```

To:
```c
esp_http_client_config_t config = {
    .url = url_buffer,
    .event_handler = _http_event_handler,
    .crt_bundle_attach = esp_crt_bundle_attach,  // NEW
};
```

**In main/CMakeLists.txt:**

Add the required components:
```cmake
idf_component_register(SRCS "your_sources.c" ...
                    INCLUDE_DIRS "."
                    REQUIRES esp_http_client mbedtls esp-tls  # Add these
                    EMBED_TXTFILES "cert.pem")  # Can remove this line now
```

### Pros:
- **No manual certificate updates needed!**
- Works with any API that uses standard certificate authorities
- Certificates auto-renew on the server side without firmware changes

### Cons:
- Slightly larger firmware size (~10-20KB for the certificate bundle)
- Trusts all common CAs (less restrictive than pinning specific certs)

---

## When to Use Each Solution

**Use Certificate Bundle (Solution 2) if:**
- Your API uses standard SSL certificates from common CAs (Let's Encrypt, DigiCert, etc.)
- You want a "set it and forget it" solution
- The device will be deployed long-term without easy firmware updates

**Use Manual Certificates (Solution 1) if:**
- You need certificate pinning for extra security
- You're connecting to internal/self-signed certificates
- Firmware size is extremely constrained

---

## Quick Reference Commands

**Check certificate expiry:**
```bash
openssl x509 -in cert.pem -text -noout | grep -A 2 "Validity"
```

**Download fresh certificate chain:**
```bash
openssl s_client -showcerts -servername DOMAIN -connect DOMAIN:443 </dev/null 2>/dev/null | sed -n '/BEGIN CERTIFICATE/,/END CERTIFICATE/p' > cert.pem
```

**Verify certificate count:**
```bash
grep -c "BEGIN CERTIFICATE" cert.pem
```

---

## Troubleshooting

If you still get certificate errors after updating:

1. **Check ESP32's system time** - SSL certificates have validity dates
   - Look for SNTP sync messages in logs
   - Incorrect date/time will cause "certificate expired" errors

2. **Verify you have the full chain** - Should be 2-3 certificates, not just 1

3. **Check the domain matches** - Certificate must match the exact domain you're connecting to

4. **Try certificate bundle first** - Easier than manual certificate management