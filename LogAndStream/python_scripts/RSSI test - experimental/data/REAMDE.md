# Description
This python script polls the available bluetooth device for the RSSI value of nearby Shimmer devices. Currently, this script is only compatible with Linux and has only been tested on the Ubuntu platform - hence it is experimental. 

# Setting up
To use this script, you will need to install the following:

## Pip
```
sudo apt-get install python-pip python-dev build-essential ll
sudo pip install --upgrade pip 
sudo pip install --upgrade virtualenv 
```

## Bluetooth libraries
```
sudo apt-get install bluetooth libbluetooth-dev
sudo python3 -m pip install pybluez
sudo pip install pybluez
```

## Matplotlib

```
sudo apt-get install libfreetype6-dev libpng12-dev
sudo pip install matplotlib
```

## Numpy

```
sudo pip install numpy
```

# Operation
Once the required packages are installed, you should alter the shimmeRSSI.py file to only search for the mac ids of interest. Simply change the following line:
```
macIDs = ['00:06:66:72:2C:16', '00:06:66:72:3A:09', '00:06:66:46:B6:76', '00:06:66:D1:0F:96']
```
The user is required to insert the MAC ids of their own Shimmers of interest - this will allow the script to filter out other MAC ID's and include (and plot) only the RSSI's of interest.

![Plot](https://github.com/ShimmerResearch/shimmer3/blob/master/LogAndStream/python_scripts/RSSI%20test%20-%20experimental/SampleScreenshot.png)

To stop the script, you simply exit the live plot window. Once complete, the script will save timestamped RSSI values for each of the Shimmers you specify under ```macIDs``` above. These text files will be timestamped and are titled with the shortened MAC-id names of the Shimmers, for example:
```
RSSI_2C16_04-05-2017_15.59.txt
RSSI_3A09_04-05-2017_15.59.txt
RSSI_B676_04-05-2017_15.59.txt
```

These files will be stored in the ```/data``` folder.
