#
#   2 Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#   3 Neither the names of the copyright holders nor the names of the
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#   =======================================================================
script_path="$( cd "$(dirname "$0")" ; pwd -P )"

app_name="display"
remote_host=$1
download_mode=$2

presenter_atlasdk_ip=""
presenter_view_ip=""

# ************************check ip****************************************
# Description:  check ip valid or not
# $1: ip
# ******************************************************************************
function check_ip_addr()
{
    ip_addr=$1
    echo ${ip_addr} | grep "^[0-9]\{1,3\}\.\([0-9]\{1,3\}\.\)\{2\}[0-9]\{1,3\}$" > /dev/null
    if [ $? -ne 0 ]
    then
        return 1
    fi

    for num in `echo ${ip_addr} | sed "s/./ /g"`
    do
        if [ $num -gt 255 ] || [ $num -lt 0 ]
        then
            return 1
        fi
   done
   return 0
}

# ************************check ip in same segment or not****************************************
# Description:  check ip in same segment or not
# $1: checked ip
# $2: compared ip
# $3: compared net mask
# ******************************************************************************
function check_ips_in_same_segment()
{
    ip=$1
    mask=$2
    remote_host=$3
    
    OLD_IFS_IP="${IFS}"
    IFS="."
    remote_host_attr=(${remote_host})
    ip_attr=(${ip})
    mask_attr=(${mask})
    IFS=${OLD_IFS_IP}
    for i in `seq 0 3`
    do
        ((calc_remote=${remote_host_attr[${i}]}&${mask_attr[${i}]}))
        ((calc_ip=${ip_attr[${i}]}&${mask_attr[${i}]}))
        
        if [[ calc_remote -ne calc_ip ]];then
            return 1
        fi
    done
    return 0
}

# ************************convert CIDR to netmask****************************************
# Description:  convert CIDR to netmask
# $1: CIDR
# ******************************************************************************
function cidr2mask()
{
   # Number of args to shift, 255..255, first non-255 byte, zeroes
   set -- $(( 5 - ($1 / 8) )) 255 255 255 255 $(( (255 << (8 - ($1 % 8))) & 255 )) 0 0 0
   [ $1 -gt 1 ] && shift $1 || shift
   echo ${1-0}.${2-0}.${3-0}.${4-0}
}

function parse_presenter_altasdk_ip()
{
    valid_ips=""
    for ip_info in `/sbin/ip addr | grep "inet " | awk -F ' ' '{print $2}'`
    do
        ip=`echo ${ip_info} | awk -F '/' '{print $1}'`
        cidr=`echo ${ip_info} | awk -F '/' '{print $2}'`

        valid_ips="${valid_ips}\t${ip}\n"
        mask=`cidr2mask ${cidr}`
        if [[ ${ip}"X" == "X" ]];then
            continue
        fi
        check_ips_in_same_segment ${ip} ${mask} ${remote_host}
        if [[ $? -eq 0 ]];then
            presenter_atlasdk_ip=${ip}
            echo "Find ${presenter_atlasdk_ip} which is in the same segment with ${remote_host}."
            break
        fi
    done

    
    if [[ ${presenter_atlasdk_ip}"X" != "X" ]];then
        return 0
    fi
    
    echo "Can not find ip in the same segment with ${remote_host}."
    while [[ ${presenter_atlasdk_ip}"X" == "X" ]]
    do
        echo -en "Current environment valid ip list:\n${valid_ips}Please choose one which can connect to Atlas DK Developerment Board:"
        read presenter_atlasdk_ip
        if [[ ${presenter_atlasdk_ip}"X" != "X" ]];then
            check_ip_addr ${presenter_atlasdk_ip}
            if [[ $? -ne 0 ]];then
                echo "Invlid ip, please choose again..."
                presenter_atlasdk_ip=""
            else
                ret=`/sbin/ifconfig | grep ${presenter_atlasdk_ip}`
                if [[ $? -ne 0 ]];then
                    presenter_atlasdk_ip=""
                fi
            fi
        fi
    done
    return 0
}

function parse_presenter_view_ip()
{
    valid_view_ips=""
    for ip_info in `/sbin/ip addr | grep "inet " | awk -F ' ' '{print $2}'`
    do
        ip=`echo ${ip_info} | awk -F '/' '{print $1}'`
        valid_view_ips="${valid_view_ips}\t${ip}\n"
    done

    while [[ ${presenter_view_ip}"X" == "X" ]]
    do
        echo -en "Current environment valid ip list:\n${valid_view_ips}Please choose one to show the presenter in browser(default: 127.0.0.1):"
        read presenter_view_ip
        
        if [[ ${presenter_view_ip}"X" != "X" ]];then
            check_ip_addr ${presenter_view_ip}
            if [[ $? -ne 0 ]];then
                echo "Invlid ip, please choose again..."
                presenter_view_ip=""
            else
                ret=`/sbin/ifconfig | grep ${presenter_view_ip}`
                if [[ $? -ne 0 ]];then
                    echo "Invlid ip, please choose again..."
                    presenter_view_ip=""
                fi
            fi
        else
            presenter_view_ip="127.0.0.1"
        fi
    done
    return 0
}

function main()
{
    stop_pid=`ps -ef | grep "presenter_server\.py" | grep "${app_name}" | awk -F ' ' '{print $2}'`
    if [[ ${stop_pid}"X" != "X" ]];then
        echo -e "\033[33mNow do presenter server configuration, kill existing presenter process: kill -9 ${stop_pid}.\033[0m"
        kill -9 ${stop_pid}
    fi

    if [[ ${download_mode} != "local" ]];then
        echo "Install python3 libs: pip3 install -r ${script_path}/requirements..."
        pip3 install -r ${script_path}/requirements
        if [ $? -ne 0 ];then
            echo "ERROR: install python3 libs failed, please check your env."
            return 1
        fi
    fi
    
    parse_presenter_altasdk_ip
    parse_presenter_view_ip
    
    echo "Use ${presenter_atlasdk_ip} to connect to Atlas DK Developerment Board..."
    sed -i "s/presenter_server_ip=[0-9.]*/presenter_server_ip=${presenter_atlasdk_ip}/g" ${script_path}/${app_name}/config/config.conf
    
    echo "Use ${presenter_view_ip} to show information in browser..."
    sed -i "s/web_server_ip=[0-9.]*/web_server_ip=${presenter_view_ip}/g" ${script_path}/${app_name}/config/config.conf
    echo "Finish to prepare ${app_name} presenter server ip configuration."
}

main
