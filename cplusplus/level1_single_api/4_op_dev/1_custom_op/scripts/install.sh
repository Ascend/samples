#!/bin/bash

targetdir=/usr/local/Ascend/opp
target_custom=0

sourcedir=$PWD/packages

QUIET="n"

for i in "$@"
do
    echo $i
    if test $i = "--quiet"; then
        QUIET="y"
        break
    fi
done

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
        has_same_file=-1
        for file_a in ${sourcedir}/$1/*; do
            file_b=${file_a##*/};
            grep -q $file_b <<<`ls ${targetdir}/$1`;
            if [[ $? -eq 0 ]]; then
                echo -n "${file_b} "
                has_same_file=0
            fi
        done
        if [ 0 -eq $has_same_file ]; then
            if test $QUIET = "n"; then
                echo "has old version in ${targetdir}/$1"\
                "Do you want to replace? [y/n] "

                while true
                do
                    read yn
                    if [ "$yn" = n ]; then
                        return 0
                    elif [ "$yn" = y ]; then
                        break;
                    else
                        echo "[ERROR] input error, please input again!"
                    fi
                done
            fi
        fi
        log "[INFO] replace old ops $1 files ......"
    fi

    log "copy new ops $1 files ......"
    if [ -d ${targetdir}/$1/custom/ ]; then
        chmod -R +w "$targetdir/$1/custom/" >/dev/null 2>&1
    fi
    cp -rf ${sourcedir}/$1/* $targetdir/$1/
    if [ $? -ne 0 ];then
        log "[ERROR] copy new $1 files failed"
        return 1
    fi

    return 0
}
upgrade_proto()
{
    if [ ! -f ${sourcedir}/custom.proto ]; then
        log "[INFO] no need to upgrade custom.proto files"
        return 0
    fi
    if [ ! -d ${targetdir}/framework/custom/caffe ];then
        log "[INFO] create ${targetdir}/framework/custom/caffe."
        mkdir -p ${targetdir}/framework/custom/caffe
        if [ $? -ne 0 ];then
            log "[ERROR] create ${targetdir}/framework/custom/caffe failed"
            return 1
        fi
    else
        if [ -f ${targetdir}/framework/custom/caffe/custom.proto ]; then
            # 有老版本,判断是否要覆盖式安装
            if test $QUIET = "n"; then
                echo "[INFO] ${targetdir}/framework/custom/caffe has old version"\
                "custom.proto file. Do you want to replace? [y/n] "

                while true
                do
                    read yn
                    if [ "$yn" = n ]; then
                        return 0
                    elif [ "$yn" = y ]; then
                        break;
                    else
                        echo "[ERROR] input error, please input again!"
                    fi
                done
            fi
        fi
        log "[INFO] replace old caffe.proto files ......"
    fi
    chmod -R +w "$targetdir/framework/custom/caffe/" >/dev/null 2>&1
    cp -rf ${sourcedir}/custom.proto ${targetdir}/framework/custom/caffe/
    if [ $? -ne 0 ];then
        log "[ERROR] copy new custom.proto failed"
        return 1
    fi
	log "[INFO] copy custom.proto success"

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

upgrade_proto
if [ $? -ne 0 ];then
    exit 1
fi

changemode()
{
    if [ -d ${targetdir} ];then
        subdirs=$(ls "${targetdir}" 2> /dev/null)
        for dir in ${subdirs}; do
            if [[ ${dir} != "Ascend310" ]] && [[ ${dir} != "Ascend310RC" ]]&& [[ ${dir} != "Ascend910" ]] && [[ ${dir} != "Ascend710" ]] && [[ ${dir} != "Ascend310" ]] && [[ ${dir} != "aicpu" ]]; then
                chmod -R 550 "${targetdir}/${dir}" >/dev/null 2>&1
            fi
        done
    fi

    return 0
}
echo "[ops_custom]changemode..."
#changemode
if [ $? -ne 0 ];then
    exit 1
fi

if [ -d ${targetdir}/op_impl/custom/cpu/aicpu_kernel/custom_impl/ ]; then
    chmod -R 440 ${targetdir}/op_impl/custom/cpu/aicpu_kernel/custom_impl/* >/dev/null 2>&1
fi
if [ -f ${targetdir}/ascend_install.info ]; then
    chmod -R 440 ${targetdir}/ascend_install.info
fi
if [ -f ${targetdir}/scene.info ]; then
    chmod -R 440 ${targetdir}/scene.info
fi
if [ -f ${targetdir}/version.info ]; then
    chmod -R 440 ${targetdir}/version.info
fi

echo "SUCCESS"
exit 0

