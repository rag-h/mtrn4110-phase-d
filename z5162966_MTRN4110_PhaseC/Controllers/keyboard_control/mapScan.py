# create new file for testing
f = open("demofile2.txt", "w")
f.write("Now the file has content!")
f.close()



def simpleOutput(myName):
    outputString = "Hello this is a test for " + myName
    print(outputString)
    return "Hello"