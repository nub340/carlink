const char* ca_cert = R"EOF(-----BEGIN CERTIFICATE-----
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
-----END CERTIFICATE-----)EOF";

const char* client_cert = R"EOF(-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIUKK2RmQ2oxOrEH07T7v6OrdJdKKcwDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTI0MDkwNzA4NTkw
N1oXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAMXCVT3HH3E5slyb0sFq
2x82IQC94Ij5pvHPi7lAXyU5lH15PKvaV1EAFMcFT3QvP8b5ubC9qFGEGWOXERsw
8q5urMqcUlhpc6MgAQBIYvQBI6ieNo237whR9MY6EPSxQhDHn3JjO9H5GzmzNCAM
FqN4wMo09Wimm74VqXEFrfvwcdzsnYhpu/HKiKoABtGfUpHRzUExNPp68D9CVSsG
I9Oy7b9837/5QwUOxEKUVEWGBjWVfQ/04s1WELiVWlag+rvOamQpW2H5MNxtF1ap
Bxu3r16ar3VyAO2WXJ4MvpgYaIJ8EYm68RA5kNCPn6L7ZF1RBZsq9LHNu934C56M
rWsCAwEAAaNgMF4wHwYDVR0jBBgwFoAUCJ5cfEFkucULV5SO5OX/t3xY19owHQYD
VR0OBBYEFMGaBUP5a5ulQ7nQ5B6aYHMwGHUgMAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQAc/TP+G0DLZHdQfpht9t98bcgy
Ub0qE2X5XQJaMQhtD6wkaPexRHbShEi6R93hROKbaNPB0FoeAITplo3UfyaKtYwW
fSvDqRDiuE5v5iQVj1V8Sca12jcx/ctNgDM4/qxJaaxbv3Kjl0eH4Bl9chmKQ4mm
FTSvljSJoInsQIlRhcWJQAh7GiVXfzhVbLNtLC2zYEoXH76QdVtyFDjvSlg8Ingj
CdbffYksVuUFGXP5RIEPdquGBo+4J/1zE0ldgKSAb05X+4Diwx/DfiJAOY3RG1Lv
NI4mIEeN8Abhdb2xvB1yjVwm0jiGGy32F0VIkS3ttjFr8xciq6FB3VQDFWK2
-----END CERTIFICATE-----)EOF";

const char* client_key = R"EOF(-----BEGIN PRIVATE KEY-----
MIIEvQIBADANBgkqhkiG9w0BAQEFAASCBKcwggSjAgEAAoIBAQDFwlU9xx9xObJc
m9LBatsfNiEAveCI+abxz4u5QF8lOZR9eTyr2ldRABTHBU90Lz/G+bmwvahRhBlj
lxEbMPKubqzKnFJYaXOjIAEASGL0ASOonjaNt+8IUfTGOhD0sUIQx59yYzvR+Rs5
szQgDBajeMDKNPVoppu+FalxBa378HHc7J2IabvxyoiqAAbRn1KR0c1BMTT6evA/
QlUrBiPTsu2/fN+/+UMFDsRClFRFhgY1lX0P9OLNVhC4lVpWoPq7zmpkKVth+TDc
bRdWqQcbt69emq91cgDtllyeDL6YGGiCfBGJuvEQOZDQj5+i+2RdUQWbKvSxzbvd
+AuejK1rAgMBAAECggEAWq3wbkJ3HXEKY0KkUkJQNbCZU4C9i7GGJo03kXt95VDQ
0PQdqOOWdDLoVBYQR8M+qFtsFSnyG2bKSYtEscYxEWD4ZHNnoti/yIXtT5bhoOdA
1ZmuhC9aMON7rMWFbt404ALZDxeHn5FtNXkBpsxEOTMum8nK2fDDq1a9enDTRZk8
wOdpp2rMwL25rxJIUGN3pCA29Yhi8tqMXTSZ6GaTcVVcEK916816QOTfemZEE1Qq
qZtEkX/0QogeCimwlPCLMmM4QiR4UxyzSaHQG/Ka1jKxkrDj26yAaz3G3lrho9ou
gRiwlKziFznjPTEC4t3cOQm0YC/Qk9BvBoax6UgrcQKBgQD7/A8HKyFmg3G40HTf
3bpNDMmH9IRcPEg+fmftPd147L+CXnT5hzMBRYI5vfTDSj9h4xLij9Y+QPF8w8PL
Aa99wvR8GV58A4ISPODZW4ZIXclBTdZxTlqTxRIkmelRl7aF7/YulxxBm3flmkSQ
e0xA7z/dXtvLZ8MvD7VdxuNbcwKBgQDI6RFcNSenaW+c/nS+ijqIle0pueR2TjsD
8f+qnKj9Bk94g8QpJvMZflYK9oXEZ6AJ7bc5FLN4GfbmwdXoHCN3lInMCObGvG2T
OffavQkl1IkvcttxhmxDzSrLpZBxCqy2K3OmgN0a4LuvQgNUbm1FFrt88q3eC51u
9LvmCp7YKQKBgEUY2joZ/p+wvZdzOglFMNf+AKlSSJVORnU0jmUEyN1m9iDWq/bc
twTLPal9yEdg7b522O/dh18syctvlxnszWWL9ySshDc3cwxzOyj/KvRfG1fX80zA
sp/As3fr6IniMz/JDFW8RBhNfHQueEyyMre/o2CdqWG+g0w9X3tmlWO3AoGAfWuB
ut4z8v6O/qKanpgMElgFxZaWWIRdswjPYeKKW+okkkcvncEC4VdMv2tF9mX6MjiD
uM6DeuY6i7NVpaBrQ6smmoi0CDx46v+1CogXvdYADe7VrvkLfQgHPzoMAYV4bACU
Ic0wOicxAydijMKi1A5s3INipsr5bNq6sdDPeIECgYEA08dYhr9AhIgN36+7Tcwa
SVbRxnKXjh7In6/tOEtAlYvngH4JxzQYYrEA3hL61RSMEAKdDQWqwrIbRUIh6iPf
VjfOOdbSZnaBxu9LbeLzxdFbFkn2LKSNdt09Rn6HBAF7us0h8JcTLyVYBv0+w+fv
RcQ2Os5pTPoKaJxgGSTtce8=
-----END PRIVATE KEY-----)EOF";
