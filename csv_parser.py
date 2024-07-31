import csv
# import file from os
with open('D:\Hobbies\PartyTorch Project\Software\MPU-6050-with-RGB\RGB lookup table.csv','r') as csv_file:
    csv_reader = csv.reader(csv_file)

    with open('D:\Hobbies\PartyTorch Project\Software\MPU-6050-with-RGB\modifiedFile.txt','w') as new_file:
        # file_writer = file.writer(new_file)
        for line in csv_reader:
            num1 = line[0]
            num2 = line[1]
            num3 = line[2]
            myline = "{" + num1 + ", " + num2  +", " + num3 + "},\n"
            new_file.write(myline)
            # print(type(num1))