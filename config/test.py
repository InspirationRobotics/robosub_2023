import os
from json import load

result = json.load(open("/home/inspiration/auv/config/onyx.json", "r"))
print(result.get("sub"))