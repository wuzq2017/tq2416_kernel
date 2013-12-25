#!/bin/bash
#
#
KERNEL=$PWD
echo "kernel dir: $KERNEL"
# find 用法
# 只查找某个目录
# find . -p
# 想要忽略一个完整的目录树，应当使用-prune 而不是检查目录树中所有的文件。例如：要跳过
# 'src/emacs' 目录和其中所有的文件和子目录，把其他找到的文件打印出来，应当这样：
#                        find . -path './src/emacs' -prune -o -print
# -o : 逻辑或简写   expr1 -or expr2
# !expr:  逻辑非  -not expr
# generate the cscope.files
# 添加arch/arm文件夹,根据自己的cpu架构修改
# 排除/scripts, /drivers, /Documentation, /tmp

# 注意net/* 和直接net*写法
find  "$KERNEL"        \
    -path "$KERNEL/arch/*" ! -path "$KERNEL/arch/arm*" -prune -o               \
    -path "$KERNEL/arch/arm/mach-*" ! -path "$KERNEL/arch/arm/mach-s3c2416*" \
    ! -path "$KERNEL/arch/arm/mach-godarm*" -prune -o \
    -path "$KERNEL/arch/arm/plat-*" ! -path "$KERNEL/arch/arm/plat-s3c24xx*" -prune -o\
	-path "$KERNEL/tmp*" -prune -o                                           \
	-path "$KERNEL/Documentation*" -prune -o                                 \
	-path "$KERNEL/scripts*" -prune -o                                       \
	-path "$KERNEL/drivers/*" ! -path "$KERNEL/drivers/tty*" \
    ! -path "$KERNEL/drivers/mmc*" \
    ! -path "$KERNEL/drivers/input*" -prune -o  \
    -path "$KERNEL/sound*" -prune -o                                       \
    -path "$KERNEL/crypto*" -prune -o   \
    -path "$KERNEL/tools*" -prune -o   \
    -path "$KERNEL/fs*" -prune -o   \
    -path "$KERNEL/net*" -prune -o   \
    -path "$KERNEL/security*" -prune -o   \
    -path "$KERNEL/samples*" -prune -o   \
    -path "$KERNEL/mm*" -prune -o   \
    -path "$KERNEL/block*" -prune -o   \
    -path "$KERNEL/firmware*" -prune -o   \
    -path "$KERNEL/lib*" -prune -o   \
    -name "*.[chxsS]" -print > $KERNEL/gtags.files 

echo "Generated gtags.file."
# cscope -bkq -i $KERNEL/cscope.files  
gtags -f  $KERNEL/gtags.files
#generate the cppcomplete  
# ctags -n -f cppcomplete.tags --fields=+ai --C++-types=+p * -L $KERNEL/cscope.files  
#Try setting the $CSCOPE_DB environment variable to point to a Cscope database you create, so you won't al#ways need to launch Vim in the     same directory as the database.  
# export CSCOPE_DB=$KERNEL/cscope.out 
