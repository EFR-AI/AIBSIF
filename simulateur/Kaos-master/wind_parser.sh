#!/bin/bash

if [[ "$#" -lt 2 ]];
then
	echo "Please choose a wind profile name"
	exit
fi

cd wind #Changes working directory to the wind directory

rm -rf dir_"$2" #Deletes the wind profile it one with the same name exists
mkdir dir_"$2" #Makes a new wind profil with the choosen name

tail -n +6 "$1" > working_file #Removes the first 6 lines of the weather baloon file
awk '{print $1}' working_file  > alt #Takes the 1s column as the altitude and stores it in the alt file
awk '{print $5}' working_file  > dir #Same but it's the directions
awk '{print $6}' working_file  > mag #Same but it's the wind speed


mv alt dir_"$2"
mv dir dir_"$2"
mv mag dir_"$2"

rm working_file
