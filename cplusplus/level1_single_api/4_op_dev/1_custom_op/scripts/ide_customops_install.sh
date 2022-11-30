#!/bin/bash

targetdir=/usr/local/Ascend/opp
target_custom=0

sourcedir=$PWD/packages
vendordir=vendors/$vendor_name

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
    if [ ! -d ${sourcedir}/$vendordir/$1 ]; then
        log "[INFO] no need to upgrade ops $1 files"
        return 0
    fi

    if [ ! -d ${targetdir}/$vendordir/$1 ];then
        log "[INFO] create ${targetdir}/$vendordir/$1."
        mkdir -p ${targetdir}/$vendordir/$1
        if [ $? -ne 0 ];then
            log "[ERROR] create ${targetdir}/$vendordir/$1 failed"
            return 1
        fi
    else
        vendor_installed_dir=$(ls "$targetdir/vendors" 2> /dev/null)
        for i in $vendor_installed_dir;do
            vendor_installed_file=$(ls "$vendor_installed_dir/$vendor_name/$i" 2> /dev/null)
            if [ "$i" = "$vendor_name" ] && [ "$vendor_installed_file" != "" ]; then
                echo "[INFO]: $vendor_name custom opp package has been installed on the path $vendor_installed_dir:
- Overlay Installation , please enter:[o]
- Replace directory installation , please enter: [r]
- Do not install , please enter:[n]
>>>"
	    fi
            while true
            do
                read orn
                if [ "$orn" = o ]; then
                    break
                elif [ "$orn" = r ]; then
                    [ -n "$vendor_installed_file"] && rm -rf "$vendor_installed_file"
                    break
                elif [ "$orn" = n ]; then
                    return 0
                else
                    echo "[WARNING]: Input error, please input [o] or [r] or [n] to choose!"
                fi
            done
        done	
        log "[INFO] replace old ops $1 files ......"
    fi

    log "copy new ops $1 files ......"
    cp -rf ${sourcedir}/$vendordir/$1/* $targetdir/$vendordir/$1/
    if [ $? -ne 0 ];then
        log "[ERROR] copy new $1 files failed"
        return 1
    fi

    return 0
}
log "[INFO] copy uninstall sh success"

chmod u+w ${targetdir}
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

config_file=${targetdir}/vendors/config.ini

if [ ! -f ${config_file} ]; then
    touch ${config_file}
    chmod 550 ${config_file}
fi

found_vendors="$(grep -w "load_priority" "$config_file" | cut --only-delimited -d"=" -f2-)"
found_vendor=$(echo $found_vendors | sed "s/$vendor_name//g" | tr ',' ' ')
vendor=$(echo $found_vendor | tr -s ' ' ',')
if [ "$vendor" != "" ]; then
    sed -i "/load_priority=$found_vendors/s@load_priority=$found_vendors@load_priority=$vendor_name,$vendor@g" "$config_file"
fi

if [ "$vendor" == "" ]; then
    echo "load_priority=customize" > ${config_file}
fi

chmod u-w ${targetdir}
changemode()
{
    if [ -d ${targetdir} ];then
        # chmod -R 550 ${targetdir}
        subdirs=$(ls "${targetdir}" 2> /dev/null)
        for dir in ${subdirs}; do
            if [[ ${dir} != "Ascend" ]] && [[ ${dir} != "Ascend310" ]] && [[ ${dir} != "Ascend310RC" ]] && [[ ${dir} != "Ascend910" ]] && [[ ${dir} != "Ascend310P" ]]  && [[ ${dir} != "aicpu" ]]; then
                chmod -R 550 "${targetdir}/${dir}" >/dev/null 2>&1
            fi
        done
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

subdirs=$(ls "${targetdir}" 2> /dev/null)
for dir in ${subdirs}; do
    if [[ ${dir} != "Ascend" ]] && [[ ${dir} != "Ascend310" ]] && [[ ${dir} != "Ascend310RC" ]] && [[ ${dir} != "Ascend910" ]] && [[ ${dir} != "Ascend310P" ]]  && [[ ${dir} != "aicpu" ]]; then
        chmod -R -w "${targetdir}/${dir}"  >/dev/null 2>&1
    fi
done
if [ $? -ne 0 ];then
    exit 1
fi

echo "SUCCESS"
exit 0

