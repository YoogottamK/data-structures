from random import randint as rand

num = 1000
print(num)

print(num * (num - 1))

for i in range(1, num + 1):
    for j in range(1, num + 1):
        if i != j:
            print(i, j, rand(1, 1000))

