line = "GPS, 50.0000N, 4.3000E"
print(line)
print(type(line))
# Split the string using commas as separator, we get a list of strings
values = line.replace("N","").replace("E","").replace(":", ",").split(',')
print(values)
