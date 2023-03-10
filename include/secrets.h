/**
 * ESP32 secrets file
 * 
 * Contains information required to connect to WiFi and in-turn, AWS IoT
 * 
 * Authors: Vipul Deshpande, Jaime Burbano
 */


/*
  Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
  Permission is hereby granted, free of charge, to any person obtaining a copy of this
  software and associated documentation files (the "Software"), to deal in the Software
  without restriction, including without limitation the rights to use, copy, modify,
  merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#include <pgmspace.h>

#define SECRET
//#define THINGNAME "PublisherG05"
#define THINGNAME "SubscriberG05"

const char WIFI_SSID[] = "Mars";
const char WIFI_PASSWORD[] = "counterclockwise";
const char AWS_IOT_ENDPOINT[] = "a2w4pcihe6wk22-ats.iot.eu-central-1.amazonaws.com";

//Publisher Certificates
/* Amazon Root CA 1 */
static const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

//for Subscriber stuff i guess 
/* Device Certificate */
static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
MIIDWjCCAkKgAwIBAgIVAJStQp+geEnjNerRzSB8TpEZBIk5MA0GCSqGSIb3DQEB
CwUAME0xSzBJBgNVBAsMQkFtYXpvbiBXZWIgU2VydmljZXMgTz1BbWF6b24uY29t
IEluYy4gTD1TZWF0dGxlIFNUPVdhc2hpbmd0b24gQz1VUzAeFw0yMjExMjMxMDEw
NDBaFw00OTEyMzEyMzU5NTlaMB4xHDAaBgNVBAMME0FXUyBJb1QgQ2VydGlmaWNh
dGUwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDHZGUaucmvVoWJVhB8
AqCNhJDpr16xxr9WrpvQ54IPbhPxEjUtDhorU2MtuOJcMQ47DYExTZWlySlR2V+E
cTS1QOQiV9awY63dQlTmebPGiIP+Wqm90HU2X8kDxAmv6mbxasgnh9Dypu0C8L4L
//zwgctkCSdfXmow/3RCWybcCmLoMKMDfn4v8TItAA7yIV0mfOlIzGO2bpdalhsM
b0s/fELGOi0b0xWRRjdTfNcbk4AxhZ1GqCf9A52NJfHIT5T2WSZugPtAguiKNeTN
VX56Pu8zVsQRovhaeaWCEVyi/lXnK1DXZolUgEl0q/1hPgkshHDCFaNAkNWkrhpp
oN5NAgMBAAGjYDBeMB8GA1UdIwQYMBaAFFo35JFo/JfnIke9lbdSJVc0cAjLMB0G
A1UdDgQWBBSW7c7nGs9O9fXnLGqWDxL81K/gCTAMBgNVHRMBAf8EAjAAMA4GA1Ud
DwEB/wQEAwIHgDANBgkqhkiG9w0BAQsFAAOCAQEAVy59OGX4XY8rNTG8x3Szb0sX
rO6vZuQIsG59BdFlP4O3IhGxKoQj5maPFh2DNggMJOV5AYflAI2vcUnzc6EzKch8
XXR2wVK4j0Fz3rqSVz4ClVFi7VRhoDSlGWYBxZuGMjFolFOgN/YNNCaTLn4c0qYA
11NjDxL+5HceDGhw6b6UOTEObbuSlRy3qrGdS+Wl5XpLOkhYzgyfjkjkzifrZJ7E
U/38fZJHePpRXiw7Z4S3FCTJRAb/lyrWRDnSGduwdbahPbXkSNbG2YsqyHeT3xqq
G+OUyH2hiABbCi1rD0xq42bAbCbaqsth6A/7TzCJQAoeSPoKHer+qbJbmnGmyA==
-----END CERTIFICATE-----
)KEY";

/* Device Private Key */
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
MIIEowIBAAKCAQEAx2RlGrnJr1aFiVYQfAKgjYSQ6a9esca/Vq6b0OeCD24T8RI1
LQ4aK1NjLbjiXDEOOw2BMU2VpckpUdlfhHE0tUDkIlfWsGOt3UJU5nmzxoiD/lqp
vdB1Nl/JA8QJr+pm8WrIJ4fQ8qbtAvC+C//88IHLZAknX15qMP90Qlsm3Api6DCj
A35+L/EyLQAO8iFdJnzpSMxjtm6XWpYbDG9LP3xCxjotG9MVkUY3U3zXG5OAMYWd
Rqgn/QOdjSXxyE+U9lkmboD7QILoijXkzVV+ej7vM1bEEaL4WnmlghFcov5V5ytQ
12aJVIBJdKv9YT4JLIRwwhWjQJDVpK4aaaDeTQIDAQABAoIBAH12AVwYrPwjz/CD
c+nj6UxCrsArtcTczsZnJeoD0cWNuQSGNWl9JyA93vkFDYF+6Hpl2/W+4LupYNN8
mGqIpOarxyCI7rCcd+jx9OP0jhcNDSi9uXKTOT2WdbBl6MaFEnMzgm/FOMeGGzov
axW35FsdAHKYg+qxSNY91tRt8Hyg/Boi6OsZ0TOy986aC5x/0SZHA4UV2B2jnDfF
pTEHTBz3SV7+s3vmU+yLPZqrorXNxWTqiuop5364cLFGB4yO1BZ3JZ5F4fVk9lyZ
hCBcyntgaTMOAAvbn5oVwdO7/+VXZonXhfj6PPBJ4e9On0Qbs2TXtMSoYJV8nGhy
bv/0S80CgYEA4r5ScULpcAKD/87NwocEkqfzRPZ32E4HmnaCzsma346DLnPrRV7z
lXJ7a0v7HJvZUTyQqpu0y43v/bQ1bzvt9FoLJDDp8VZCJFrDfw5BAwke2lxKSGh1
n/gVPWKxk+1gH+uxxj7uhv7ph3QR5JCEtlvCdfTZk/+24aKOFtFkjHMCgYEA4R6e
czjSHZUhYzPwfgEx4CdKB5Mjwr461lr6d085YXo8AYQD7ypA1knRpL1FEU8iVx5T
03sDcSPUZj64vpOEqun/IREmX8uHrppP+13YJ1Wq/QkdfHZqja6WUdnx3GfGGrWA
+tCWhA3E+zhwiXL1TvaOx0RFvhpcR1m++Ufe+j8CgYEA2cFD9af3/e08jYAzVP1L
4/hNhc5ZvC772Fi2OReel9IKo6MJsJ6GueCvLJVUvN3jb8QFfwN4NwDCGVlm88OP
O9r51jDviyl27ekI1M2AnAS3T6fQYweG5zFeXIya0+DlLuKw4R3w/05yQfazdFli
SquG7rdN+XdcUAdIBBvmP4cCgYBlWYSaeS1Xdilm4R5XjDApK2cPy/WgmkYg9Sqo
BjneIcrcH9Qpm4s/LlK86R0sMsVSFWxF3K7rA+d+9KP14LIsxFZBS9hL7nSybfSu
RUyoQNBPRQhKXYGRMdWpGgbiZLHyml5sDPaGkzBorbs0llXMbAJRwfHoKTP8R3JM
BkhjvwKBgBWPN3neHuvtRE5Gdz+hIkJFQtBXpMAqGtP+LesaPgNhMNlOkQ1daGMS
A3poot+VqUDztYsQwesH3uBmt2F2vlWEi/zmI3bXVDm2Hc6EITYq8vyYirbZaS9l
8DrLuUltOxD1TWUP6oRELf0BbCHn+Loiz3QdFAvT2VDXjJHzoBaU
-----END RSA PRIVATE KEY-----
)KEY";
