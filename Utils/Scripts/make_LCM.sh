#!/bin/bash


GREEN='\033[0;32m'
NC='\033[0m' # No Color

echo -e "${GREEN} Starting LCM type generation...${NC}"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd ${DIR}/../../LCM-Types
# Clean
rm */*.jar
rm */*.java
rm */*.hpp
rm */*.class
rm */*.py
rm */*.pyc
rm */*.cs
rm ../DynamicsEngine/Unity_Visualization/Assets/LCM-Types/*.cs
rm ../DynamicsEngine/Unity_Visualization/Assets/LCM-Types/*.cs.meta

# Make
ls *.lcm
lcm-gen -jxp --cpp-std=c++11 *.lcm
lcm-gen --csharp --csharp-strip-dirs *.lcm
cp /usr/share/java/lcm.jar .
cp /usr/local/share/java/lcm.jar .
javac -cp lcm.jar */*.java
jar cf my_types.jar */*.class
mkdir -p java
mv my_types.jar java
mv lcm.jar java
mkdir -p cpp
mv *.hpp cpp

mkdir -p csharp
mv *.cs csharp

cp csharp/*.cs ../DynamicsEngine/Unity_Visualization/Assets/LCM-Types

mkdir -p python
mv *.py python

FILES=$(ls */*.class)
echo ${FILES} > file_list.txt


echo -e "${GREEN} Done with LCM type generation${NC}"
