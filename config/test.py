import os
from dotenv import dotenv_values

result = dotenv_values("onyx.env")

print(result.get("sub"))