cmd_drivers/media/video/samsung/ump/../mali/linux/mali_osk_math.o := /home/yyoung.kim/Toolchain/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/arm-eabi-gcc -Wp,-MD,drivers/media/video/samsung/ump/../mali/linux/.mali_osk_math.o.d  -nostdinc -isystem /home/yyoung.kim/Toolchain/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/../lib/gcc/arm-eabi/4.4.3/include -I/home/yyoung.kim/KOR_ANDR_OSS_OPEN_M0SKT-FINAL/Kernel/arch/arm/include -Iarch/arm/include/generated -Iinclude  -include include/generated/autoconf.h -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-exynos/include -Iarch/arm/plat-s5p/include -Iarch/arm/plat-samsung/include -Wall -Wundef -Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -Werror-implicit-function-declaration -Wno-format-security -fno-delete-null-pointer-checks -O2 -fdiagnostics-show-option -Werror -Wno-error=unused-function -Wno-error=unused-variable -Wno-error=unused-value -Wno-error=unused-label -marm -fno-dwarf2-cfi-asm -fno-omit-frame-pointer -mapcs -mno-sched-prolog -mabi=aapcs-linux -mno-thumb-interwork -D__LINUX_ARM_ARCH__=7 -march=armv7-a -msoft-float -Uarm -Wframe-larger-than=1024 -fno-stack-protector -fno-omit-frame-pointer -fno-optimize-sibling-calls -g -pg -Wdeclaration-after-statement -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Idrivers/media/video/samsung/ump/ -Idrivers/media/video/samsung/ump/common -Idrivers/media/video/samsung/ump/linux -Idrivers/media/video/samsung/ump/include -Idrivers/media/video/samsung/ump/linux/license/gpl/ -Idrivers/media/video/samsung/ump/../mali/common -Idrivers/media/video/samsung/ump/../mali/linux -DKERNEL_BUILTIN=1 -DMALI_USE_UNIFIED_MEMORY_PROVIDER -DUSING_MEMORY=1 -DUMP_MEM_SIZE=1024 -DMALI_STATE_TRACKING=1 -DSVN_REV= -DSVN_REV_STRING=\"\"    -D"KBUILD_STR(s)=\#s" -D"KBUILD_BASENAME=KBUILD_STR(mali_osk_math)"  -D"KBUILD_MODNAME=KBUILD_STR(ump)" -c -o drivers/media/video/samsung/ump/../mali/linux/mali_osk_math.o drivers/media/video/samsung/ump/../mali/linux/mali_osk_math.c

source_drivers/media/video/samsung/ump/../mali/linux/mali_osk_math.o := drivers/media/video/samsung/ump/../mali/linux/mali_osk_math.c

deps_drivers/media/video/samsung/ump/../mali/linux/mali_osk_math.o := \
  drivers/media/video/samsung/ump/../mali/common/mali_osk.h \
  include/linux/types.h \
    $(wildcard include/config/uid16.h) \
    $(wildcard include/config/lbdaf.h) \
    $(wildcard include/config/arch/dma/addr/t/64bit.h) \
    $(wildcard include/config/phys/addr/t/64bit.h) \
    $(wildcard include/config/64bit.h) \
  /home/yyoung.kim/KOR_ANDR_OSS_OPEN_M0SKT-FINAL/Kernel/arch/arm/include/asm/types.h \
  include/asm-generic/int-ll64.h \
  /home/yyoung.kim/KOR_ANDR_OSS_OPEN_M0SKT-FINAL/Kernel/arch/arm/include/asm/bitsperlong.h \
  include/asm-generic/bitsperlong.h \
  include/linux/posix_types.h \
  include/linux/stddef.h \
  include/linux/compiler.h \
    $(wildcard include/config/sparse/rcu/pointer.h) \
    $(wildcard include/config/trace/branch/profiling.h) \
    $(wildcard include/config/profile/all/branches.h) \
    $(wildcard include/config/enable/must/check.h) \
    $(wildcard include/config/enable/warn/deprecated.h) \
  include/linux/compiler-gcc.h \
    $(wildcard include/config/arch/supports/optimized/inlining.h) \
    $(wildcard include/config/optimize/inlining.h) \
  include/linux/compiler-gcc4.h \
  /home/yyoung.kim/KOR_ANDR_OSS_OPEN_M0SKT-FINAL/Kernel/arch/arm/include/asm/posix_types.h \
  drivers/media/video/samsung/ump/../mali/common/mali_kernel_memory_engine.h \
  drivers/media/video/samsung/ump/../mali/linux/mali_osk_specific.h \
  include/linux/bitops.h \
  /home/yyoung.kim/KOR_ANDR_OSS_OPEN_M0SKT-FINAL/Kernel/arch/arm/include/asm/bitops.h \
    $(wildcard include/config/smp.h) \
  /home/yyoung.kim/KOR_ANDR_OSS_OPEN_M0SKT-FINAL/Kernel/arch/arm/include/asm/system.h \
    $(wildcard include/config/function/graph/tracer.h) \
    $(wildcard include/config/cpu/32v6k.h) \
    $(wildcard include/config/cpu/xsc3.h) \
    $(wildcard include/config/cpu/fa526.h) \
    $(wildcard include/config/arch/has/barriers.h) \
    $(wildcard include/config/arm/dma/mem/bufferable.h) \
    $(wildcard include/config/cpu/sa1100.h) \
    $(wildcard include/config/cpu/sa110.h) \
    $(wildcard include/config/cpu/v6.h) \
  include/linux/linkage.h \
  /home/yyoung.kim/KOR_ANDR_OSS_OPEN_M0SKT-FINAL/Kernel/arch/arm/include/asm/linkage.h \
  include/linux/irqflags.h \
    $(wildcard include/config/trace/irqflags.h) \
    $(wildcard include/config/irqsoff/tracer.h) \
    $(wildcard include/config/preempt/tracer.h) \
    $(wildcard include/config/trace/irqflags/support.h) \
  include/linux/typecheck.h \
  /home/yyoung.kim/KOR_ANDR_OSS_OPEN_M0SKT-FINAL/Kernel/arch/arm/include/asm/irqflags.h \
  /home/yyoung.kim/KOR_ANDR_OSS_OPEN_M0SKT-FINAL/Kernel/arch/arm/include/asm/ptrace.h \
    $(wildcard include/config/cpu/endian/be8.h) \
    $(wildcard include/config/arm/thumb.h) \
  /home/yyoung.kim/KOR_ANDR_OSS_OPEN_M0SKT-FINAL/Kernel/arch/arm/include/asm/hwcap.h \
  /home/yyoung.kim/KOR_ANDR_OSS_OPEN_M0SKT-FINAL/Kernel/arch/arm/include/asm/outercache.h \
    $(wildcard include/config/outer/cache/sync.h) \
    $(wildcard include/config/outer/cache.h) \
  include/asm-generic/cmpxchg-local.h \
  include/asm-generic/bitops/non-atomic.h \
  include/asm-generic/bitops/fls64.h \
  include/asm-generic/bitops/sched.h \
  include/asm-generic/bitops/hweight.h \
  include/asm-generic/bitops/arch_hweight.h \
  include/asm-generic/bitops/const_hweight.h \
  include/asm-generic/bitops/lock.h \
  include/asm-generic/bitops/le.h \
  /home/yyoung.kim/KOR_ANDR_OSS_OPEN_M0SKT-FINAL/Kernel/arch/arm/include/asm/byteorder.h \
  include/linux/byteorder/little_endian.h \
  include/linux/swab.h \
  /home/yyoung.kim/KOR_ANDR_OSS_OPEN_M0SKT-FINAL/Kernel/arch/arm/include/asm/swab.h \
  include/linux/byteorder/generic.h \

drivers/media/video/samsung/ump/../mali/linux/mali_osk_math.o: $(deps_drivers/media/video/samsung/ump/../mali/linux/mali_osk_math.o)

$(deps_drivers/media/video/samsung/ump/../mali/linux/mali_osk_math.o):
