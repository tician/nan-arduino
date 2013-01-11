#! /usr/bin/env python3.1

import re
from optparse import OptionParser
from datetime import datetime


parser = OptionParser()
parser.add_option(  "-i", "--infile", dest="inFile",
                    help="Name of input file", metavar="FILE",
                    action="store", type="string")
parser.add_option(  "-o", "--outfile", dest="outFile",
                    help="Name of output file", metavar="FILE",
                    action="store", type="string")
parser.add_option(  "-d", "--head", dest="headName",
                    help="Header's #ifndef/#def", metavar="string",
                    action="store", type="string")

(options, args) = parser.parse_args()
inFile = options.inFile
outFile = options.outFile
headName = options.headName

if (not inFile or len(inFile)<4):
    inFile=input("Please enter the name (and path if not in current"+
                 " directory)of the input '.mtn' file\n")
    if (len(inFile)<4):
        print("Wow.  The village callled...")
        exit()

if (not outFile or len(outFile)<4):
    outFile=inFile[:-3]+'h'

if (not headName or len(headName)<4):
    dt=datetime.utcnow()
    headName = "POKEY_"+str(dt.year)+str(dt.month)+str(dt.day)+str(dt.hour)+str(dt.minute)+str(dt.second)



postype = "PROGMEM prog_uint16_t "
seqtype = "PROGMEM transition_t "

templist = []
pmetlist = []

rawlist = []
valid=0

poselist = []
seqlist = []

# open the input '.mtn' file and parse it a bit

moFi = open(inFile, 'r')

for line in moFi:
    if (re.match("name=[a-zA-Z0-9][a-zA-Z0-9]*", line) != None):
        valid=1
        templist.append(line[5:-1].replace(' ', ''))
    elif ((re.match("step=", line) != None) and (valid==1)):
        temp=line[5:-1].replace(' ', ',', 25)
        temp=temp.replace(' ', '};\t//wait:', 1)
        temp=temp.replace(' ', '\t//move:', 1)
        templist.append(temp)
    elif ((re.match("page_end", line) != None) and (valid==1)):
        rawlist.append(templist[:])
        del templist[:]
        valid=0

moFi.close()

#print(len(rawlist))
#print(rawlist)
#print(rawlist[0][1])

# run through the raw strings recovered from the input file and
#  finish cleanup/parsing

for index in range(len(rawlist)):
    for pose in range(len(rawlist[index])):
        if (pose==0):
            del templist[:]
            del pmetlist[:]
            templist.append(seqtype+rawlist[index][0]+'[] = {{0,'+str(len(rawlist[index])-1)+'}')
        if (pose>0):
            pmetlist.append(postype+rawlist[index][0]+'_'+str(pose)+'[] = {0x1A,'+rawlist[index][pose]+'\n')
            sought=re.search("//move:.*", rawlist[index][pose])
            temp=sought.group(0)
            temp=temp.replace('.', '')
            templist.append(',{'+rawlist[index][0]+'_'+str(pose)+','+temp[7:]+'}')
            if (pose==len(rawlist[index])-1):
                templist.append('};')
                newstr = ''
                seqlist.append(newstr.join(templist))
                newerstr=''
                poselist.append(newerstr.join(pmetlist))


#print(len(rawlist[0]))
#print(rawlist[0][1])
#print(templist)
#print(seqlist)
#print(poselist)
#print(pmetlist)


poFi = open(outFile, 'w')

poFi.write('#ifndef '+headName+'\n#define '+headName+'\n\n#include <avr/pgmspace.h>\n\n\n')

for index in range(len(rawlist)):
    poFi.write('// Sequence: '+rawlist[index][0]+'\n')
#    for pose in range(len(rawlist[index])-1):
#        poFi.write(poselist[index][pose]+'\n')
    poFi.write(poselist[index])
    poFi.write('\n'+seqlist[index]+'\n\n\n')

poFi.write('#endif\n')
poFi.close()



#POSES=$( cat "$INNAME" \
#| sed -ne "/name=..*/{s/ //g; s:name:PROGMEM prog_uint16_t :1; s|=\(..*\)|\1_[] =\t\{0x1A,|1; h;}; /step=/{s/step=//1; s: [^ ]\.[^ ]* :}\;\/\/:1; s/ /,/g; x; P; s:\[\] :_\[\] :1; x; p; };" \
#| sed -ne "/PROGMEM/{N; s/,\n/,/1; p;}" \
#| sed -ne "s/_________\[/_9[/1; s/________\[/_8[/1; s/_______\[/_7[/1; s/______\[/_6[/1; s/_____\[/_5[/1; s/____\[/_4[/1; s/___\[/_3[/1; s/__\[/_2[/1; s:\(PROGMEM prog_uint16_t ..*_\)\[:\n\n//########\n\11[:1; p;" \
#| sed -ne ":PROGMEM.*:{s:PROGMEM prog_uint16_t \(.*\)\[\] =.*//\([0-9]\.[0-9]*\):,{\1,\2}:1; H; :;\n:{N; :;\n\n:{G;p;};};};" \
#)


#echo -e "#ifndef "$HEADNAME"\n#define "$HEADNAME"\n\n#include <avr/pgmspace.h>\n" > "$OUTNAME"
#echo "$POSES" >> "$OUTNAME"
#echo -e "\n\n#endif" >> "$OUTNAME"
