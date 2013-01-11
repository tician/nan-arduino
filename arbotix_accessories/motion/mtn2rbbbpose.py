#! /usr/bin/env python3.1

#

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
parser.add_option(  "-s", "--suffix", dest="seqSuffix",
                    help="Suffix for sequence names", metavar="string",
                    action="store", type="string")
parser.add_option(  "-p", "--prefix", dest="seqPrefix",
                    help="Prefix for sequence names", metavar="string",
                    action="store", type="string")
parser.add_option(  "-O", "--size", dest="sizeOptimize", default=False,
                    help="Only use one servo ID array per processed motion file",
                    action="store_true")



(options, args) = parser.parse_args()
inFile = options.inFile
outFile = options.outFile
headName = options.headName
seqPrefix = options.seqPrefix
seqSuffix = options.seqSuffix
sizeOptimize = options.sizeOptimize


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
else:
    headName=headName.upper()
    if (headName[-1]!='H' and headName[-2]!='_'):
        headName=headName+'_H'

if (not seqPrefix):
    seqPrefix=''
if (not seqSuffix):
    seqSuffix=''

postype = "PROGMEM prog_uint16_t "
seqtype = "PROGMEM transition_t "

templist = []
pmetlist = []
splitlist = []
punklist = []

rawlist = []
valid=0

poselist = []
seqlist = []
servolist = []

activeServos = []
numActiveServos = 0


# open the input '.mtn' file and parse it a bit

moFi = open(inFile, 'r')

for line in moFi:
    if (re.match("name=[a-zA-Z0-9][a-zA-Z0-9]*", line) != None):
        valid=1
        templist.append(line[5:-1].replace(' ', ''))
    elif (re.match("enable=", line) !=None):
        temp=line[7:-1]
        splitlist=re.split(" ", temp)
        for index in range(26):
            activeServos.append(int(splitlist[index]))
            if (activeServos[index]==1):
                numActiveServos+=1
    elif ((re.match("step=", line) != None) and (valid==1)):
        temp=line[5:-1]
        templist.append(temp)
    elif ((re.match("page_end", line) != None) and (valid==1)):
        rawlist.append(templist[:])
        del templist[:]
        valid=0

moFi.close()

#print(len(rawlist))
#print(rawlist)
#print(rawlist[0][1])


if (sizeOptimize):
    punklist.append(postype+seqPrefix+outFile[:-2]+seqSuffix+'_id[] = {'+str(numActiveServos))
    for indice in range(25):
        if (activeServos[indice]==1):
            punklist.append(','+str(indice))
    punklist.append('};')
    neweststr=''
    servolist.append(neweststr.join(punklist))


# run through the raw strings recovered from the input file and
#  finish cleanup/parsing

for index in range(len(rawlist)):
    for pose in range(len(rawlist[index])):
        if (pose==0):
            del templist[:]
            del pmetlist[:]
            del punklist[:]

            if (not sizeOptimize):
                punklist.append(postype+seqPrefix+rawlist[index][0]+seqSuffix+'_id[] = {'+str(numActiveServos))
                for indice in range(25):
                   if (activeServos[indice]==1):
                        punklist.append(','+str(indice))
                punklist.append('};')
                templist.append(seqtype+seqPrefix+rawlist[index][0]+seqSuffix+'[] = {{'+seqPrefix+rawlist[index][0]+seqSuffix+'_id,'+str(len(rawlist[index])-1)+'}')
            else:
                templist.append(seqtype+seqPrefix+rawlist[index][0]+seqSuffix+'[] = {{'+seqPrefix+outFile[:-2]+seqSuffix+'_id,'+str(len(rawlist[index])-1)+'}')

        if (pose>0):
            pmetlist.append(postype+seqPrefix+rawlist[index][0]+seqSuffix+'_'+str(pose)+'[] = {'+str(numActiveServos))
            splitlist=re.split(" ", rawlist[index][pose])

            for indice in range(25):
                if (activeServos[indice]==1):
                    pmetlist.append(','+str(splitlist[indice]))
            pmetlist.append('};\t//wait:'+str(splitlist[26])+'\t//move:'+str(splitlist[27])+'\n')
            
            # add pose array pointer and time to pose in sequence array
            templist.append(',{'+seqPrefix+rawlist[index][0]+seqSuffix+'_'+str(pose)+','+str(int(float(splitlist[27])*1000))+'}')

            # if the end of the array, close braces
            if (pose==len(rawlist[index])-1):
                templist.append('};')
                newstr = ''
                seqlist.append(newstr.join(templist))
                newerstr=''
                poselist.append(newerstr.join(pmetlist))
                if (not sizeOptimize):
                    neweststr=''
                    servolist.append(neweststr.join(punklist))


#print(servolist)
#print(templist)
#print(seqlist)
#print(poselist)
#print(pmetlist)
#exit()


poFi = open(outFile, 'w')

poFi.write('#ifndef '+headName+'\n#define '+headName+'\n\n#include <avr/pgmspace.h>\n\n\n')

if (not sizeOptimize):
    for index in range(len(rawlist)):
        poFi.write('// Sequence: '+rawlist[index][0]+'\n')
        poFi.write(poselist[index]+'\n')
        poFi.write(servolist[index]+'\n')
        poFi.write(seqlist[index]+'\n\n\n')
else:
    poFi.write('// Size Optimized with one servo ID array per processed motion file\n')
    poFi.write('// Motion file: '+seqPrefix+outFile[:-2]+seqSuffix+'\n')
    poFi.write(servolist[0]+'\n\n\n\n')
    for index in range(len(rawlist)):
        poFi.write('// Sequence: '+rawlist[index][0]+'\n')
        poFi.write(poselist[index]+'\n')
        poFi.write(seqlist[index]+'\n\n\n')


poFi.write('#endif\n')
poFi.close()

exit()


#The initial bash script with sed (~4 hours wasted)

#POSES=$( cat "$INNAME" \
#| sed -ne "/name=..*/{s/ //g; s:name:PROGMEM prog_uint16_t :1; s|=\(..*\)|\1_[] =\t\{0x1A,|1; h;}; /step=/{s/step=//1; s: [^ ]\.[^ ]* :}\;\/\/:1; s/ /,/g; x; P; s:\[\] :_\[\] :1; x; p; };" \
#| sed -ne "/PROGMEM/{N; s/,\n/,/1; p;}" \
#| sed -ne "s/_________\[/_9[/1; s/________\[/_8[/1; s/_______\[/_7[/1; s/______\[/_6[/1; s/_____\[/_5[/1; s/____\[/_4[/1; s/___\[/_3[/1; s/__\[/_2[/1; s:\(PROGMEM prog_uint16_t ..*_\)\[:\n\n//########\n\11[:1; p;" \
#| sed -ne ":PROGMEM.*:{s:PROGMEM prog_uint16_t \(.*\)\[\] =.*//\([0-9]\.[0-9]*\):,{\1,\2}:1; H; :;\n:{N; :;\n\n:{G;p;};};};" \
#)


#echo -e "#ifndef "$HEADNAME"\n#define "$HEADNAME"\n\n#include <avr/pgmspace.h>\n" > "$OUTNAME"
#echo "$POSES" >> "$OUTNAME"
#echo -e "\n\n#endif" >> "$OUTNAME"
