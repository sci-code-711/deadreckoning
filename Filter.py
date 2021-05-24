import csv
import pandas as pd
import math as M
import numpy as np

data = [] # create array in which data will be stored
with open("Example_data.csv") as csvfile:
    data = pd.read_csv(csvfile, delimiter=',',skiprows=[2]) # opens file for use and enters it into array

leng=data.shape[0] # extracts parameter for length of data table

filt_data=pd.DataFrame(index=range(leng),columns=range(7)) # generates new dataframe to save filetred values to
filt_data.columns=["t","ax","ay","az","vl","vm","vn"]

i=2 # specifies number of bins either side to avaerage over
print(2*i+1)

filt_collums=[1,2,3,4,5,6]
unfilt_collums=[0]

for c in unfilt_collums:
    for r in range(0,leng):
        filt_data.iloc[r,c]=data.iloc[r,c]

for c in filt_collums:
    tot=0
    for r in range(0,leng):
        if r==0:
            for a in range(0,i+1):
                tot=tot+data.iloc[a,c]
            filt_data.iloc[0,c]=tot/(i+1)
        elif r<i+1:
            tot=tot+data.iloc[r+i,c]
            filt_data.iloc[r,c]=tot/(r+i+1)
        elif r>leng-i-1:
            tot=tot-data.iloc[r-i-1,c]
            filt_data.iloc[r,c]=tot/(leng-r+i)
        else:
            tot=tot+data.iloc[r+i,c]-data.iloc[r-i-1,c]
            filt_data.iloc[r,c]=tot/(2*i+1)
                
filt_data.to_csv('Temp_filt_data.csv', index=False) # writes data with calibrated gyroscope readings to new tempory file

print("filtering complete")