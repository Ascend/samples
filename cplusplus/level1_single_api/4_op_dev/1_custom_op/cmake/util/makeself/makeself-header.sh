cat << EOF  > "$archname"
#!/bin/sh
# This script was generated using Makeself $MS_VERSION
# The license covering this archive and its contents, if any, is wholly independent of the Makeself license (GPL)

ORIG_UMASK=\`umask\`
if test "$KEEP_UMASK" = n; then
    umask 077
fi

OPP_CUSTOM_VENDOR="$OPP_CUSTOM_VENDOR"
CRCsum="$CRCsum"
MD5="$MD5sum"
SHA="$SHAsum"
TMPROOT=\${TMPDIR:=\$PWD}
USER_PWD="\$PWD"; export USER_PWD
name_of_file="\$0 "
pwd_of_file="\$PWD "


label="$LABEL"
script="$SCRIPT"
scriptargs="$SCRIPTARGS"
licensetxt="$LICENSE"
helpheader='$HELPHEADER'
targetdir="$archdirname"
filesizes="$filesizes"
keep="n"
nooverwrite="$NOOVERWRITE"
quiet="n"
uninstall="n"
accept="n"
nodiskspace="n"
export_conf="$EXPORT_CONF"

print_cmd_arg=""
if type printf > /dev/null; then
    print_cmd="printf"
elif test -x /usr/ucb/echo; then
    print_cmd="/usr/ucb/echo"
else
    print_cmd="echo"
fi

if test -d /usr/xpg4/bin; then
    PATH=/usr/xpg4/bin:\$PATH
    export PATH
fi

if test -d /usr/sfw/bin; then
    PATH=\$PATH:/usr/sfw/bin
    export PATH
fi

unset CDPATH

MS_Printf()
{
    \$print_cmd \$print_cmd_arg "\$1"
}

MS_PrintLicense()
{
  if test x"\$licensetxt" != x; then
    echo "\$licensetxt" | more
    if test x"\$accept" != xy; then
      while true
      do
        MS_Printf "Please type y to accept, n otherwise: "
        read yn
        if test x"\$yn" = xn; then
          keep=n
          eval \$finish; exit 1
          break;
        elif test x"\$yn" = xy; then
          break;
        fi
      done
    fi
  fi
}

MS_diskspace()
{
	(
	df -kP "\$1" | tail -1 | awk '{ if (\$4 ~ /%/) {print \$3} else {print \$4} }'
	)
}

MS_dd()
{
    blocks=\`expr \$3 / 1024\`
    bytes=\`expr \$3 % 1024\`
    dd if="\$1" ibs=\$2 skip=1 obs=1024 conv=sync 2> /dev/null | \\
    { test \$blocks -gt 0 && dd ibs=1024 obs=1024 count=\$blocks ; \\
      test \$bytes  -gt 0 && dd ibs=1 obs=1024 count=\$bytes ; } 2> /dev/null
}

MS_dd_Progress()
{
    if test x"\$noprogress" = xy; then
        MS_dd \$@
        return \$?
    fi
    file="\$1"
    offset=\$2
    length=\$3
    pos=0
    bsize=4194304
    while test \$bsize -gt \$length; do
        bsize=\`expr \$bsize / 4\`
    done
    blocks=\`expr \$length / \$bsize\`
    bytes=\`expr \$length % \$bsize\`
    (
        dd ibs=\$offset skip=1 2>/dev/null
        pos=\`expr \$pos \+ \$bsize\`
        MS_Printf "     0%% " 1>&2
        if test \$blocks -gt 0; then
            while test \$pos -le \$length; do
                dd bs=\$bsize count=1 2>/dev/null
                pcent=\`expr \$length / 100\`
                pcent=\`expr \$pos / \$pcent\`
                if test \$pcent -lt 100; then
                    MS_Printf "\b\b\b\b\b\b\b" 1>&2
                    if test \$pcent -lt 10; then
                        MS_Printf "    \$pcent%% " 1>&2
                    else
                        MS_Printf "   \$pcent%% " 1>&2
                    fi
                fi
                pos=\`expr \$pos \+ \$bsize\`
            done
        fi
        if test \$bytes -gt 0; then
            dd bs=\$bytes count=1 2>/dev/null
        fi
        MS_Printf "\b\b\b\b\b\b\b" 1>&2
        MS_Printf " 100%%  " 1>&2
    ) < "\$file"
}

MS_Help()
{
    cat << EOH >&2
 1) Getting help or info about \$0 :
  \$0 -h |--help   Print this message
  \$0 --info   Print embedded info : title, default target directory, embedded script ...
  \$0 --lsm    Print embedded lsm entry (or no LSM)
  \$0 --list   Print the list of files in the archive
  \$0 --check  Checks integrity of the archive
  \$0 --uninstall  Uninstall product

 2) Running \$0 :
  \$0 [options] [--] [additional arguments to embedded script]
  with following options (in that order)
  --confirm             Ask before running embedded script
  --quiet               Do not print anything except error messages
  --install-path=<path> Install package to specific dir path
  --accept              Accept the license
  --noexec              Do not run embedded script
  --target dir          Extract directly to a target directory (absolute or relative)
                        This directory may undergo recursive chown (see --nochown).
  --keep                Do not erase target directory after running the embedded script
  --noprogress          Do not show the progress during the decompression
  --nox11               Do not spawn an xterm
  --nochown             Do not give the extracted files to the current user
  --nodiskspace         Do not check for available disk space
  --tar arg1 [arg2 ...] Access the contents of the archive through the tar command
  --  arg1 [arg2 ...]   Following arguments will be passed to the embedded script.These can be used:
\${helpheader}
EOH
}

MS_Check()
{
    OLD_PATH="\$PATH"
    PATH=\${GUESS_MD5_PATH:-"\$OLD_PATH:/bin:/usr/bin:/sbin:/usr/local/ssl/bin:/usr/local/bin:/opt/openssl/bin"}
	MD5_ARG=""
    MD5_PATH=\`exec <&- 2>&-; which md5sum || command -v md5sum || type md5sum\`
    test -x "\$MD5_PATH" || MD5_PATH=\`exec <&- 2>&-; which md5 || command -v md5 || type md5\`
    test -x "\$MD5_PATH" || MD5_PATH=\`exec <&- 2>&-; which digest || command -v digest || type digest\`
    PATH="\$OLD_PATH"

    SHA_PATH=\`exec <&- 2>&-; which shasum || command -v shasum || type shasum\`
    test -x "\$SHA_PATH" || SHA_PATH=\`exec <&- 2>&-; which sha256sum || command -v sha256sum || type sha256sum\`

    if test x"\$quiet" = xn; then
		MS_Printf "Verifying archive integrity..."
    fi
    offset=\`head -n $SKIP "\$1" | wc -c | tr -d " "\`
    verb=\$2
    i=1
    for s in \$filesizes
    do
		crc=\`echo \$CRCsum | cut -d" " -f\$i\`
		if test -x "\$SHA_PATH"; then
			if test x"\`basename \$SHA_PATH\`" = xshasum; then
				SHA_ARG="-a 256"
			fi
			sha=\`echo \$SHA | cut -d" " -f\$i\`
			if test x"\$sha" = x0000000000000000000000000000000000000000000000000000000000000000; then
				test x"\$verb" = xy
			else
				shasum=\`MS_dd_Progress "\$1" \$offset \$s | eval "\$SHA_PATH \$SHA_ARG" | cut -b-64\`;
				if test x"\$shasum" != x"\$sha"; then
					echo "Error in SHA256 checksums: \$shasum is different from \$sha" >&2
					exit 2
				else
					test x"\$verb" = xy && MS_Printf " SHA256 checksums are OK." >&2
				fi
				crc="0000000000";
			fi
		fi
		if test -x "\$MD5_PATH"; then
			if test x"\`basename \$MD5_PATH\`" = xdigest; then
				MD5_ARG="-a md5"
			fi
			md5=\`echo \$MD5 | cut -d" " -f\$i\`
			if test x"\$md5" = x00000000000000000000000000000000; then
				test x"\$verb" = xy
			else
				md5sum=\`MS_dd_Progress "\$1" \$offset \$s | eval "\$MD5_PATH \$MD5_ARG" | cut -b-32\`;
				if test x"\$md5sum" != x"\$md5"; then
					echo "Error in MD5 checksums: \$md5sum is different from \$md5" >&2
					exit 2
				else
					test x"\$verb" = xy && MS_Printf " MD5 checksums are OK." >&2
				fi
				crc="0000000000"; verb=n
			fi
		fi
		if test x"\$crc" = x0000000000; then
			test x"\$verb" = xy
		else
			sum1=\`MS_dd_Progress "\$1" \$offset \$s | CMD_ENV=xpg4 cksum | awk '{print \$1}'\`
			if test x"\$sum1" = x"\$crc"; then
				test x"\$verb" = xy && MS_Printf " CRC checksums are OK." >&2
			else
				echo "Error in checksums: \$sum1 is different from \$crc" >&2
				exit 2;
			fi
		fi
		i=\`expr \$i + 1\`
		offset=\`expr \$offset + \$s\`
    done
    if test x"\$quiet" = xn; then
		echo " All good."
    fi
}

MS_Uninstall()
{
    if test x"\$OPP_CUSTOM_VENDOR" = xcustomize; then
        rm -rf \${ASCEND_OPP_PATH}/vendors/$OPP_CUSTOM_VENDOR/op_impl
        rm -rf \${ASCEND_OPP_PATH}/vendors/$OPP_CUSTOM_VENDOR/framework
        rm -rf \${ASCEND_OPP_PATH}/vendors/$OPP_CUSTOM_VENDOR/op_proto
        if [ ! -d "\${ASCEND_OPP_PATH}/vendors/$OPP_CUSTOM_VENDOR/op_impl" ] && [ ! -d "\${ASCEND_OPP_PATH}/vendors/$OPP_CUSTOM_VENDOR/framework" ] && [ ! -d "\${ASCEND_OPP_PATH}/vendors/$OPP_CUSTOM_VENDOR/op_proto" ];then
            echo "uninstall SUCCESS."
        else
            echo "uninstall FAIL."
        fi
    else
        rm -rf \${ASCEND_OPP_PATH}/vendors/$OPP_CUSTOM_VENDOR
        if [ ! -d "\${ASCEND_OPP_PATH}/vendors/$OPP_CUSTOM_VENDOR" ];then
            echo "uninstall SUCCESS."
        else
            echo "uninstall FAIL."
        fi
    fi
}

UnTAR()
{
    if test x"\$quiet" = xn; then
		tar \$1vf - $UNTAR_EXTRA 2>&1 || { echo " ... Extraction failed." > /dev/tty; kill -15 \$$; }
    else
		tar \$1f - $UNTAR_EXTRA 2>&1 || { echo Extraction failed. > /dev/tty; kill -15 \$$; }
    fi
}

Check_Path_Exist_Permission()
{
    # root
    prepare_check_path=\$1
    if [ "\$(id -u)" = "0" ]; then
        sh -c "test -d \${prepare_check_path} 2> /dev/null"
        if [ \$? -eq 0 ]; then
            sh -c "test -w \${prepare_check_path} 2> /dev/null"
            if [ \$? -eq 0 ]; then
                return 0
            else
                echo "[ERROR] user do access \${prepare_check_path} failed, please check the path permission"
                return 2
            fi
        else
            echo "[DEBUG] the \${prepare_check_path} does not exist"
            return 1
        fi
    # not root
    else
        test -d \${prepare_check_path} >> /dev/null 2>&1
        if [ \$? -eq 0 ]; then
            test -w \${prepare_check_path} >> /dev/null 2>&1
            if [ \$? -eq 0 ]; then
                return 0
            else
                echo "[ERROR] user do access \${prepare_check_path} failed, please check the path permission"
                return 2
            fi
        else
            return 3
        fi
    fi
}

Check_Install_Path()
{
    in_install_path_param="\$1"
    param_name="\$2"

    # empty patch check
    if [ "x\${in_install_path_param}" = "x" ]; then
        echo "[ERROR] parameter \${param_name} not support that the install path is empty"
        exit 1
    fi

    # delete last "/"
    temp_path="\${in_install_path_param}"
    temp_path=\$(echo "\${temp_path}" | sed "s/\/*$//g")
    if [ x"\${temp_path}" = "x" ]; then
        temp_path="/"
    fi

    # covert relative path to absolute path
    run_path=\$(pwd)
    prefix=\$(echo "\${temp_path}" | cut -d"/" -f1 |cut -d"~" -f1)
    if [ x"\${prefix}" = "x" ]; then
        in_install_path_param="\${temp_path}"
    else
        prefix=\$(echo "\${run_path}" | cut -d"/" -f1 |cut -d"~" -f1)
        if [ x"\${prefix}" = "x" ]; then
            in_install_path_param="\${run_path}/\${temp_path}"
        else
            echo "[ERROR] run package path is invalid : \${run_path}"
            exit 1
        fi
    fi

    # covert '~' to home path
    home=\$(echo "\${temp_path}" | cut -d"~" -f1)
    if [ "x\${home}" = "x" ]; then
        temp_path_value=\$(echo "\${temp_path}" | cut -d"~" -f2-)
        if [ "\$(id -u)" -eq 0 ]; then
            in_install_path_param="/root\$temp_path_value"
        else
            home_path=\$(eval echo "~\${USER}")
            home_path=\$(echo "\${home_path}" | sed "s/\/*$//g")
            in_install_path_param="\$home_path\$temp_path_value"
        fi
    fi

    # check whether path valid and permission
    penultimate_install_path=\$(dirname "\${in_install_path_param}")
    Check_Path_Exist_Permission \${in_install_path_param}
    # last path permission denied
    last_status=\$?
    if [ \${last_status} -eq 2 ]; then
        exit 1
    # last path does not exist or permission denied
    elif [ \${last_status} -eq 1 ] || [ \${last_status} -eq 3 ]; then
        # penultimate path does not exist or permission denied
        Check_Path_Exist_Permission \${penultimate_install_path}
        penultimate_status=\$?
        if [ \${penultimate_status} -eq 1 ] || [ \${penultimate_status} -eq 2 ]; then
            exit 1
        elif [ \${penultimate_status} -eq 3 ]; then
            echo "[ERROR] user do access \${in_install_path_param} failed, please check the path valid and permission"
            exit 1
        fi
    fi
}


finish=true
xterm_loop=
noprogress=$NOPROGRESS
nox11=$NOX11
copy=$COPY
ownership=y
verbose=n

initargs="\$@"

while true
do
    case "\$1" in
    -h | --help)
	MS_Help
	exit 0
	;;
    -q | --quiet)
	quiet=y
	noprogress=y
	shift
	;;
	--accept)
	accept=y
	shift
	;;
    --info)
	echo Identification: "\$label"
	echo Target directory: "\$targetdir"
	echo Uncompressed size: $USIZE KB
	echo Compression: $COMPRESS
	echo Date of packaging: $DATE
	echo Built with Makeself version $MS_VERSION on $OSTYPE
	echo Build command was: "$MS_COMMAND"
	if test x"\$script" != x; then
	    echo Script run after extraction:
	    echo "    " \$script \$scriptargs
	fi
	if test x"$copy" = xcopy; then
		echo "Archive will copy itself to a temporary location"
	fi
	if test x"$NEED_ROOT" = xy; then
		echo "Root permissions required for extraction"
	fi
	if test x"$KEEP" = xy; then
	    echo "directory \$targetdir is permanent"
	else
	    echo "\$targetdir will be removed after extraction"
	fi
	exit 0
	;;
    --dumpconf)
	echo LABEL=\"\$label\"
	echo SCRIPT=\"\$script\"
	echo SCRIPTARGS=\"\$scriptargs\"
	echo archdirname=\"$archdirname\"
	echo KEEP=$KEEP
	echo NOOVERWRITE=$NOOVERWRITE
	echo COMPRESS=$COMPRESS
	echo filesizes=\"\$filesizes\"
	echo CRCsum=\"\$CRCsum\"
	echo MD5sum=\"\$MD5\"
	echo OLDUSIZE=$USIZE
	echo OLDSKIP=`expr $SKIP + 1`
	exit 0
	;;
    --lsm)
cat << EOLSM
EOF
eval "$LSM_CMD"
cat << EOF  >> "$archname"
EOLSM
	exit 0
	;;
    --list)
	echo Target directory: \$targetdir
	offset=\`head -n $SKIP "\$0" | wc -c | tr -d " "\`
	for s in \$filesizes
	do
	    MS_dd "\$0" \$offset \$s | eval "$GUNZIP_CMD" | UnTAR t
	    offset=\`expr \$offset + \$s\`
	done
	exit 0
	;;
	--tar)
	offset=\`head -n $SKIP "\$0" | wc -c | tr -d " "\`
	arg1="\$2"
    if ! shift 2; then MS_Help; exit 1; fi
	for s in \$filesizes
	do
	    MS_dd "\$0" \$offset \$s | eval "$GUNZIP_CMD" | tar "\$arg1" - "\$@"
	    offset=\`expr \$offset + \$s\`
	done
	exit 0
	;;
    --check)
	MS_Check "\$0" y
	exit 0
	;;
    --install-path=*)
    is_install_path=y
    install_path=\$(echo \$1 | cut -d"=" -f2-)
    # check path
    Check_Install_Path "\${install_path}" "--install-path"
    shift
    ;;
    --uninstall)
        MS_Uninstall "\$0" y
        exit 0
        ;;
    --confirm)
	verbose=y
	shift
	;;
	--noexec)
	script=""
	shift
	;;
    --keep)
	keep=y
	shift
	;;
    --target)
	keep=y
	targetdir="\${2:-.}"
    if ! shift 2; then MS_Help; exit 1; fi
	;;
    --noprogress)
	noprogress=y
	shift
	;;
    --nox11)
	nox11=y
	shift
	;;
    --nochown)
	ownership=n
	shift
	;;
    --nodiskspace)
	nodiskspace=y
	shift
	;;
    --xwin)
	if test "$NOWAIT" = n; then
		finish="echo Press Return to close this window...; read junk"
	fi
	xterm_loop=1
	shift
	;;
    --phase2)
	copy=phase2
	shift
	;;
    --)
	shift
	break ;;
    -*)
	echo Unrecognized flag : "\$1" >&2
	MS_Help
	exit 1
	;;
    *)
	break ;;
    esac
done

quiet_para=""

if test x"\$quiet" = xy; then 
    quiet_para="--quiet "
fi

install_path_para=""

if test x"\$is_install_path" = xy; then
    install_path_para="--install-path=\$in_install_path_param"
fi

keep_para=""

if test x"\$keep" = xy; then 
    keep_para="--keep "
fi

confirm_para=""

if test x"\$verbose" = xy; then 
    confirm_para="--confirm "
fi

scriptargs="\$scriptargs""--\$name_of_file""--\$pwd_of_file""\$quiet_para""\$keep_para""\$confirm_para""\$install_path_para"
if test x"\$quiet" = xy -a x"\$verbose" = xy; then
	echo Cannot be verbose and quiet at the same time. >&2
	exit 1
fi

if test x"$NEED_ROOT" = xy -a \`id -u\` -ne 0; then
	echo "Administrative privileges required for this archive (use su or sudo)" >&2
	exit 1	
fi

if test x"\$copy" \!= xphase2; then
    MS_PrintLicense
fi

case "\$copy" in
copy)
    tmpdir="\$TMPROOT"/makeself.\$RANDOM.\`date +"%y%m%d%H%M%S"\`.\$\$
    mkdir "\$tmpdir" || {
	echo "Could not create temporary directory \$tmpdir" >&2
	exit 1
    }
    SCRIPT_COPY="\$tmpdir/makeself"
    echo "Copying to a temporary location..." >&2
    cp "\$0" "\$SCRIPT_COPY"
    chmod +x "\$SCRIPT_COPY"
    cd "\$TMPROOT"
    exec "\$SCRIPT_COPY" --phase2 -- \$initargs
    ;;
phase2)
    finish="\$finish ; rm -rf \`dirname \$0\`"
    ;;
esac

if test x"\$nox11" = xn; then
    if tty -s; then                 # Do we have a terminal?
	:
    else
        if test x"\$DISPLAY" != x -a x"\$xterm_loop" = x; then  # No, but do we have X?
            if xset q > /dev/null 2>&1; then # Check for valid DISPLAY variable
                GUESS_XTERMS="xterm gnome-terminal rxvt dtterm eterm Eterm xfce4-terminal lxterminal kvt konsole aterm terminology"
                for a in \$GUESS_XTERMS; do
                    if type \$a >/dev/null 2>&1; then
                        XTERM=\$a
                        break
                    fi
                done
                chmod a+x \$0 || echo Please add execution rights on \$0
                if test \`echo "\$0" | cut -c1\` = "/"; then # Spawn a terminal!
                    exec \$XTERM -title "\$label" -e "\$0" --xwin "\$initargs"
                else
                    exec \$XTERM -title "\$label" -e "./\$0" --xwin "\$initargs"
                fi
            fi
        fi
    fi
fi

if test x"\$targetdir" = x.; then
    tmpdir="."
else
    if test x"\$keep" = xy; then
	if test x"\$nooverwrite" = xy && test -d "\$targetdir"; then
            echo "Target directory \$targetdir already exists, aborting." >&2
            exit 1
	fi
	if test x"\$quiet" = xn; then
	    echo "Creating directory \$targetdir" >&2
	fi
	tmpdir="\$targetdir"
	dashp="-p"
    else
	tmpdir="\$TMPROOT/selfgz\$\$\$RANDOM"
	dashp=""
    fi
    mkdir \$dashp "\$tmpdir" || {
	echo 'Cannot create target directory' \$tmpdir >&2
	echo 'You should try option --target dir' >&2
	eval \$finish
	exit 1
    }
fi

location="\`pwd\`"
if test x"\$SETUP_NOCHECK" != x1; then
    MS_Check "\$0"
fi
offset=\`head -n $SKIP "\$0" | wc -c | tr -d " "\`

if test x"\$verbose" = xy; then
	MS_Printf "About to extract $USIZE KB in \$tmpdir ... Proceed ? [Y/n] "
	read yn
	if test x"\$yn" = xn; then
		eval \$finish; exit 1
	fi
fi

if test x"\$quiet" = xn; then
	MS_Printf "Uncompressing \$label"
	
    # Decrypting with openssl will ask for password,
    # the prompt needs to start on new line
	if test x"$ENCRYPT" = xy; then
	    echo
	fi
fi
res=3
if test x"\$keep" = xn; then
    trap 'echo Signal caught, cleaning up >&2; cd \$TMPROOT; /bin/rm -rf "\$tmpdir"; eval \$finish; exit 15' 1 2 3 15
fi

if test x"\$nodiskspace" = xn; then
    leftspace=\`MS_diskspace "\$tmpdir"\`
    if test -n "\$leftspace"; then
        if test "\$leftspace" -lt $USIZE; then
            echo
            echo "Not enough space left in "\`dirname \$tmpdir\`" (\$leftspace KB) to decompress \$0 ($USIZE KB)" >&2
            echo "Use --nodiskspace option to skip this check and proceed anyway" >&2
            if test x"\$keep" = xn; then
                echo "Consider setting TMPDIR to a directory with more free space."
            fi
            eval \$finish; exit 1
        fi
    fi
fi

for s in \$filesizes
do
    if MS_dd_Progress "\$0" \$offset \$s | eval "$GUNZIP_CMD" | ( cd "\$tmpdir"; umask \$ORIG_UMASK ; UnTAR xp ) 1>/dev/null; then
		if test x"\$ownership" = xy; then
			(cd "\$tmpdir"; chown -R \`id -u\` .;  chgrp -R \`id -g\` .)
		fi
    else
		echo >&2
		echo "Unable to decompress \$0" >&2
		eval \$finish; exit 1
    fi
    offset=\`expr \$offset + \$s\`
done
if test x"\$quiet" = xn; then
	echo
fi

cd "\$tmpdir"
res=0
if test x"\$script" != x; then
    if test x"\$export_conf" = x"y"; then
        MS_BUNDLE="\$0"
        MS_LABEL="\$label"
        MS_SCRIPT="\$script"
        MS_SCRIPTARGS="\$scriptargs"
        MS_ARCHDIRNAME="\$archdirname"
        MS_KEEP="\$KEEP"
        MS_NOOVERWRITE="\$NOOVERWRITE"
        MS_COMPRESS="\$COMPRESS"
        export MS_BUNDLE MS_LABEL MS_SCRIPT MS_SCRIPTARGS
        export MS_ARCHDIRNAME MS_KEEP MS_NOOVERWRITE MS_COMPRESS
    fi

    if test x"\$verbose" = x"y"; then
        yn="x"
        while test x"\$yn" != x -a x"\$yn" != xy -a x"\$yn" != xY -a x"\$yn" != xn -a x"\$yn" != xN
        do
            MS_Printf "OK to execute: \$script \$scriptargs \$* ? [Y/n] "
            read yn
            if test x"\$yn" = x -o x"\$yn" = xy -o x"\$yn" = xY; then
                eval "\"\$script\" \$scriptargs \"\\\$@\""; res=\$?;
            elif  test x"\$yn" = xn -o x"\$yn" = xN; then
                echo "Unable to decompress \$script ,because of aborting! ";res=\$?
            else
                echo "Input value is unacceptable,please try again."
            fi
        done
    else
		eval "\"\$script\" \$scriptargs \"\\\$@\""; res=\$?
    fi
    if test "\$res" -ne 0; then
		test x"\$verbose" = xy && echo "The program '\$script' returned an error code (\$res)" >&2
    fi
fi
if test x"\$keep" = xn; then
    cd "\$TMPROOT"
    /bin/rm -rf "\$tmpdir"
fi
eval \$finish; exit \$res
EOF
