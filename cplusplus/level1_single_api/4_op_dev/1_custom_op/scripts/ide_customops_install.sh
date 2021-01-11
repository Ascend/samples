#!/bin/bash

targetdir=/usr/local/Ascend/opp
target_custom=0

sourcedir=$PWD/packages

log() {
    cur_date=`date +"%Y-%m-%d %H:%M:%S"`
    echo "[runtime] [$cur_date] "$1
}

if [[ "x${ASCEND_OPP_PATH}" == "x" ]];then
    log "[ERROR] env ASCEND_OPP_PATH no exist"
    exit 1
fi

targetdir=${ASCEND_OPP_PATH}

if [ ! -d $targetdir ];then
    log "[ERROR] $targetdir no exist"
    exit 1
fi

chmod -R +w $targetdir
if [ $? -ne 0 ];then
    exit 1
fi

upgrade()
{
    if [ ! -d ${sourcedir}/$1 ]; then
        log "[INFO] no need to upgrade ops $1 files"
        return 0
    fi

    if [ ! -d ${targetdir}/$1 ];then
        log "[INFO] create ${targetdir}/$1."
        mkdir -p ${targetdir}/$1
        if [ $? -ne 0 ];then
            log "[ERROR] create ${targetdir}/$1 failed"
            return 1
        fi
    else
        log "[INFO] replace old ops $1 files ......"
    fi

    log "copy new ops $1 files ......"
    cp -rf ${sourcedir}/$1/* $targetdir/$1/
    if [ $? -ne 0 ];then
        log "[ERROR] copy new $1 files failed"
        return 1
    fi

    return 0
}
log "[INFO] copy uninstall sh success"

echo "[ops_custom]upgrade framework"
upgrade framework
if [ $? -ne 0 ];then
    exit 1
fi

echo "[ops_custom]upgrade op proto"
upgrade op_proto
if [ $? -ne 0 ];then
    exit 1
fi

echo "[ops_custom]upgrade op impl"
upgrade op_impl
if [ $? -ne 0 ];then
    exit 1
fi

changemode()
{
    if [ -d ${targetdir} ];then
        chmod -R 550 ${targetdir}
    fi

    if [ $? -ne 0 ];then
        log "[ERROR] chmod failed."
    fi

    return 0
}
echo "[ops_custom]changemode..."
changemode
if [ $? -ne 0 ];then
    exit 1
fi

chmod -R -w ${targetdir}
if [ $? -ne 0 ];then
    exit 1
fi

echo "SUCCESS"
exit 0

