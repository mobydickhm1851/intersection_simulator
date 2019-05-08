a = 10

def reuturn_a():
    a_p = a
    return a_p

def change_a():
    global a
    a += 1

print reuturn_a()
change_a()
print reuturn_a()

