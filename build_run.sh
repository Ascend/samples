#!/bin/bash
install_scripts(){
    lines=7
    tail -n +$lines $0 >${HOME}/samples.tar.gz
    tar jxvf ${HOME}/samples.tar.gz
    rm ${HOME}/samples.tar.gz
    exit 0
}

tar jcvf ${HOME}/samples.tar.bz2 ../samples
sed -n "1p;3,7p" ./${0}  > ./install.sh
sed 's/^[ \t]*//g' ./install.sh > soft_install.sh
cat ./soft_install.sh ${HOME}/samples.tar.bz2 > ${HOME}/samples.run
rm ./install.sh ./soft_install.sh ${HOME}/samples.tar.bz2


