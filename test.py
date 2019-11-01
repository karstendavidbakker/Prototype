line = "GPS, 5000.00N, 430.00E"
print(line)
print(type(line))
print(line[7])
# Split the string using commas as separator, we get a list of strings
values = line.replace("N","").replace("E","").replace(".","")
values2 = values[:7]+"."+values[7:14]+"."+values[14:]
values2.replace(":", ",").split(',')
print(values2)
#result = "GPS, 50.0000, 4.3000"
