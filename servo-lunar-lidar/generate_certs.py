import json
import re

def format_pem(pem_str):
    # Ensure proper PEM formatting
    if "-----BEGIN" not in pem_str:
        pem_str = "-----BEGIN CERTIFICATE-----\n" + pem_str
    if "-----END" not in pem_str:
        pem_str = pem_str + "\n-----END CERTIFICATE-----"
    return pem_str.replace("\\n", "\n").strip()

with open('data/secrets.json') as f:
    secrets = json.load(f)

with open('include/certs.h', 'w') as f:
    f.write('#ifndef CERTS_H\n#define CERTS_H\n\n')
    f.write(f'const char* root_ca = R"({format_pem(secrets["AWS_ROOT_CA"])})";\n')
    f.write(f'const char* client_cert = R"({format_pem(secrets["AWS_CERT"])})";\n')
    f.write(f'const char* client_key = R"({format_pem(secrets["AWS_KEY"])})";\n')
    f.write('\n#endif\n')