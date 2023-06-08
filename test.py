import platform
cams = ["1", "2", "3"]

for i in enumerate(cams):
    print(i[0])

print(platform.node())

if("nano" in platform.node()):
    print("true")
