import os, sys, gdb

CFG_PHY_UBOOT_BASE      = 0x33e00000
MMU_TABLE_BASE		= (CFG_PHY_UBOOT_BASE + 0xb0000 - 0x4000)
MMU_IO_AREA_START	= 0x00000000
MMU_RESERVED1_START	= 0x60000000
MMU_MEM_AREA_START	= 0xc0000000
MMU_RESERVED2_START	= 0xc4000000

def SECTION_ENTRY(base,ap,d,c,b):
    return ((base << 20)|(ap<<10)|(d<<5)|(1<<4)|(c<<3)|(b<<2)|(1<<1))

addr = [i for i in range(0,0x10000)]
def make_mmu_table ():
	mmu_addr = MMU_TABLE_BASE;
#	/* 1:1 mapping */
	for i in range((MMU_IO_AREA_START>>20),(MMU_RESERVED1_START>>20)):
		addr[i] = SECTION_ENTRY(i,3,0,0,0)
#                print ("0x%x ----> 0x%x" %(i,addr[i]))
                mmu_addr = MMU_TABLE_BASE + 4*i
                cmd = 'monitor MemU32 0x%x = 0x%x' %(mmu_addr,addr[i])
#                print cmd
                gdb.execute(cmd)
#	/* disabled */
	for i in range((MMU_RESERVED1_START>>20),(MMU_MEM_AREA_START>>20)):
            mmu_addr = MMU_TABLE_BASE + 4*i
            addr[i] = 0x00000000
            cmd = 'monitor MemU32 0x%x = 0x%x' %(mmu_addr,addr[i])
#            print cmd
            gdb.execute(cmd)



#	/* mapping system memory to 0xc0000000 */
	for i in range((MMU_MEM_AREA_START>>20),(MMU_RESERVED2_START>>20)):
            mmu_addr = MMU_TABLE_BASE + 4*i
            addr[i] = SECTION_ENTRY((i-(0xc00-0x300)),3,0,1,1)
            cmd = 'monitor MemU32 0x%x = 0x%x' %(mmu_addr,addr[i])
#            print cmd
            gdb.execute(cmd)


#	/* disabled */
	for i in range((MMU_RESERVED2_START>>20), 0x1000):
            mmu_addr = MMU_TABLE_BASE + 4*i
            addr[i] = 0x00000000
            cmd = 'monitor MemU32 0x%x = 0x%x' %(mmu_addr,addr[i])
#            print cmd
            gdb.execute(cmd)

#make_mmu_table()

class make_mmu_tlb(gdb.Command):
    """create mmu table for arm"""
    def __init__(self):
        gdb.Command.__init__(self, "make_mmu_tlb", gdb.COMMAND_DATA, gdb.COMPLETE_SYMBOL, True)

    def invoke(self, arg, from_tty):
        make_mmu_table()
        
make_mmu_tlb()
