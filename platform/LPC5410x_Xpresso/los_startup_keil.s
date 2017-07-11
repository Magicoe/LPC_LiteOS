        PRESERVE8

Stack_Size  EQU     0x00000400
Heap_Size   EQU     0x00000200

    AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem SPACE   Stack_Size
__initial_sp

    AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem    SPACE   Heap_Size
__heap_limit

    AREA    RESET, CODE, READONLY
    THUMB

    ;IMPORT  ||Image$$ARM_LIB_STACKHEAP$$ZI$$Limit||
    IMPORT  PendSV_Handler
    IMPORT  SysTick_Handler;SysTick_Handler

    EXPORT  __Vectors
    EXPORT  Reset_Handler

__Vectors
    DCD     __initial_sp
    DCD     Reset_Handler
    DCD     NMI_Handler            ; NMI Handler
    DCD     HardFault_Handler      ; Hard Fault Handler
    DCD     MemManage_Handler      ; MPU Fault Handler
    DCD     BusFault_Handler       ; Bus Fault Handler
    DCD     UsageFault_Handler     ; Usage Fault Handler
    DCD     0                      ; Checksum of the first 7 words
    DCD     0                      ; Reserved
    DCD     0                      ; Enhanced image marker, set to 0x0 for legacy boot
    DCD     0                      ; Pointer to enhanced boot block, set to 0x0 for legacy boot
    DCD     0                      ; SVCall Handler
    DCD     0                      ; Debug Monitor Handler
    DCD     0                      ; Reserved
    DCD     PendSV_Handler         ; PendSV Handler
    DCD     SysTick_Handler        ; SysTick_Handler

; External Interrupts
    DCD     WDT_IRQHandler         ; Windowed watchdog timer, Brownout detect
    DCD     BOD_IRQHandler         ; BOD interrupt
    DCD     Reserved18_IRQHandler  ; Reserved interrupt
    DCD     DMA0_IRQHandler        ; DMA controller
    DCD     GINT0_IRQHandler       ; GPIO group 0
    DCD     PIN_INT0_IRQHandler    ; Pin interrupt 0 or pattern match engine slice 0
    DCD     PIN_INT1_IRQHandler    ; Pin interrupt 1or pattern match engine slice 1
    DCD     PIN_INT2_IRQHandler    ; Pin interrupt 2 or pattern match engine slice 2
    DCD     PIN_INT3_IRQHandler    ; Pin interrupt 3 or pattern match engine slice 3
    DCD     UTICK0_IRQHandler      ; Micro-tick Timer
    DCD     MRT0_IRQHandler        ; Multi-rate timer
    DCD     CTIMER0_IRQHandler     ; Standard counter/timer CTIMER0
    DCD     CTIMER1_IRQHandler     ; Standard counter/timer CTIMER1
    DCD     CTIMER2_IRQHandler     ; Standard counter/timer CTIMER2
    DCD     CTIMER3_IRQHandler     ; Standard counter/timer CTIMER3
    DCD     CTIMER4_IRQHandler     ; Standard counter/timer CTIMER4
    DCD     SCT0_IRQHandler        ; SCTimer/PWM
    DCD     USART0_IRQHandler      ; USART0
    DCD     USART1_IRQHandler      ; USART1
    DCD     USART2_IRQHandler      ; USART2
    DCD     USART3_IRQHandler      ; USART3
    DCD     I2C0_IRQHandler        ; I2C0
    DCD     I2C1_IRQHandler        ; I2C1
    DCD     I2C2_IRQHandler        ; I2C2
    DCD     SPI0_IRQHandler        ; SPI0
    DCD     SPI1_IRQHandler        ; SPI1
    DCD     ADC0_SEQA_IRQHandler   ; ADC0 sequence A completion.
    DCD     ADC0_SEQB_IRQHandler   ; ADC0 sequence B completion.
    DCD     ADC0_THCMP_IRQHandler  ; ADC0 threshold compare and error.
    DCD     RTC_IRQHandler         ; RTC alarm and wake-up interrupts
    DCD     Reserved46_IRQHandler  ; Reserved interrupt
    DCD     MAILBOX_IRQHandler     ; Mailbox interrupt (present on selected devices)
    DCD     GINT1_IRQHandler       ; GPIO group 1
    DCD     PIN_INT4_IRQHandler    ; Pin interrupt 4 or pattern match engine slice 4 int
    DCD     PIN_INT5_IRQHandler    ; Pin interrupt 5 or pattern match engine slice 5 int
    DCD     PIN_INT6_IRQHandler    ; Pin interrupt 6 or pattern match engine slice 6 int
    DCD     PIN_INT7_IRQHandler    ; Pin interrupt 7 or pattern match engine slice 7 int
    DCD     Reserved53_IRQHandler  ; Reserved interrupt
    DCD     Reserved54_IRQHandler  ; Reserved interrupt
    DCD     Reserved55_IRQHandler  ; Reserved interrupt
    DCD     RIT_IRQHandler         ; Repetitive Interrupt Timer


;     <h> Code Read Protection level (CRP)
;       <o>    CRP_Level:
;                       <0xFFFFFFFF=> Disabled
;                       <0x4E697370=> NO_ISP
;                       <0x12345678=> CRP1
;                       <0x87654321=> CRP2
;                       <0x43218765=> CRP3 (Are you sure?)
;     </h>
CRP_Level       EQU     0xFFFFFFFF

                IF      :LNOT::DEF:NO_CRP
                AREA    |.ARM.__at_0x02FC|, CODE, READONLY
CRP_Key         DCD     0xFFFFFFFF
                ENDIF   


                AREA    |.text|, CODE, READONLY

cpu_id          EQU     0xE000ED00
cpu_ctrl        EQU     0x40000300
coproc_boot     EQU     0x40000304
coproc_stack    EQU     0x40000308

rel_vals
                DCD     cpu_id, cpu_ctrl, coproc_boot, coproc_stack
                DCW     0xFFF, 0xC24


; Dummy Exception Handlers (infinite loops which can be modified)
NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP

HardFault_Handler \
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP

MemManage_Handler     PROC
                EXPORT  MemManage_Handler         [WEAK]
                B       .
                ENDP

BusFault_Handler PROC
                EXPORT  BusFault_Handler          [WEAK]
                B       .
                ENDP

UsageFault_Handler PROC
                EXPORT  UsageFault_Handler        [WEAK]
                B       .
                ENDP

SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP

DebugMon_Handler PROC
                EXPORT  DebugMon_Handler          [WEAK]
                B       .
                ENDP

;PendSV_Handler  PROC
;                EXPORT  PendSV_Handler            [WEAK]
;                B       .
;                ENDP

;SysTick_Handler PROC
;                EXPORT  SysTick_Handler           [WEAK]
;                B       .
;                ENDP

WDT_IRQHandler\
                PROC
                EXPORT     WDT_IRQHandler        [WEAK]
                LDR        R0, =WDT_DriverIRQHandler
                BX         R0
                ENDP

BOD_IRQHandler\
                PROC
                EXPORT     BOD_IRQHandler        [WEAK]
                LDR        R0, =BOD_DriverIRQHandler
                BX         R0
                ENDP

Reserved18_IRQHandler\
                PROC
                EXPORT     Reserved18_IRQHandler        [WEAK]
                LDR        R0, =Reserved18_DriverIRQHandler
                BX         R0
                ENDP

DMA0_IRQHandler\
                PROC
                EXPORT     DMA0_IRQHandler        [WEAK]
                LDR        R0, =DMA0_DriverIRQHandler
                BX         R0
                ENDP

GINT0_IRQHandler\
                PROC
                EXPORT     GINT0_IRQHandler        [WEAK]
                LDR        R0, =GINT0_DriverIRQHandler
                BX         R0
                ENDP

PIN_INT0_IRQHandler\
                PROC
                EXPORT     PIN_INT0_IRQHandler        [WEAK]
                LDR        R0, =PIN_INT0_DriverIRQHandler
                BX         R0
                ENDP

PIN_INT1_IRQHandler\
                PROC
                EXPORT     PIN_INT1_IRQHandler        [WEAK]
                LDR        R0, =PIN_INT1_DriverIRQHandler
                BX         R0
                ENDP

PIN_INT2_IRQHandler\
                PROC
                EXPORT     PIN_INT2_IRQHandler        [WEAK]
                LDR        R0, =PIN_INT2_DriverIRQHandler
                BX         R0
                ENDP

PIN_INT3_IRQHandler\
                PROC
                EXPORT     PIN_INT3_IRQHandler        [WEAK]
                LDR        R0, =PIN_INT3_DriverIRQHandler
                BX         R0
                ENDP

UTICK0_IRQHandler\
                PROC
                EXPORT     UTICK0_IRQHandler        [WEAK]
                LDR        R0, =UTICK0_DriverIRQHandler
                BX         R0
                ENDP

MRT0_IRQHandler\
                PROC
                EXPORT     MRT0_IRQHandler        [WEAK]
                LDR        R0, =MRT0_DriverIRQHandler
                BX         R0
                ENDP

CTIMER0_IRQHandler\
                PROC
                EXPORT     CTIMER0_IRQHandler        [WEAK]
                LDR        R0, =CTIMER0_DriverIRQHandler
                BX         R0
                ENDP

CTIMER1_IRQHandler\
                PROC
                EXPORT     CTIMER1_IRQHandler        [WEAK]
                LDR        R0, =CTIMER1_DriverIRQHandler
                BX         R0
                ENDP

CTIMER2_IRQHandler\
                PROC
                EXPORT     CTIMER2_IRQHandler        [WEAK]
                LDR        R0, =CTIMER2_DriverIRQHandler
                BX         R0
                ENDP

CTIMER3_IRQHandler\
                PROC
                EXPORT     CTIMER3_IRQHandler        [WEAK]
                LDR        R0, =CTIMER3_DriverIRQHandler
                BX         R0
                ENDP

CTIMER4_IRQHandler\
                PROC
                EXPORT     CTIMER4_IRQHandler        [WEAK]
                LDR        R0, =CTIMER4_DriverIRQHandler
                BX         R0
                ENDP

SCT0_IRQHandler\
                PROC
                EXPORT     SCT0_IRQHandler        [WEAK]
                LDR        R0, =SCT0_DriverIRQHandler
                BX         R0
                ENDP

USART0_IRQHandler\
                PROC
                EXPORT     USART0_IRQHandler        [WEAK]
                LDR        R0, =USART0_DriverIRQHandler
                BX         R0
                ENDP

USART1_IRQHandler\
                PROC
                EXPORT     USART1_IRQHandler        [WEAK]
                LDR        R0, =USART1_DriverIRQHandler
                BX         R0
                ENDP

USART2_IRQHandler\
                PROC
                EXPORT     USART2_IRQHandler        [WEAK]
                LDR        R0, =USART2_DriverIRQHandler
                BX         R0
                ENDP

USART3_IRQHandler\
                PROC
                EXPORT     USART3_IRQHandler        [WEAK]
                LDR        R0, =USART3_DriverIRQHandler
                BX         R0
                ENDP

I2C0_IRQHandler\
                PROC
                EXPORT     I2C0_IRQHandler        [WEAK]
                LDR        R0, =I2C0_DriverIRQHandler
                BX         R0
                ENDP

I2C1_IRQHandler\
                PROC
                EXPORT     I2C1_IRQHandler        [WEAK]
                LDR        R0, =I2C1_DriverIRQHandler
                BX         R0
                ENDP

I2C2_IRQHandler\
                PROC
                EXPORT     I2C2_IRQHandler        [WEAK]
                LDR        R0, =I2C2_DriverIRQHandler
                BX         R0
                ENDP

SPI0_IRQHandler\
                PROC
                EXPORT     SPI0_IRQHandler        [WEAK]
                LDR        R0, =SPI0_DriverIRQHandler
                BX         R0
                ENDP

SPI1_IRQHandler\
                PROC
                EXPORT     SPI1_IRQHandler        [WEAK]
                LDR        R0, =SPI1_DriverIRQHandler
                BX         R0
                ENDP

ADC0_SEQA_IRQHandler\
                PROC
                EXPORT     ADC0_SEQA_IRQHandler        [WEAK]
                LDR        R0, =ADC0_SEQA_DriverIRQHandler
                BX         R0
                ENDP

ADC0_SEQB_IRQHandler\
                PROC
                EXPORT     ADC0_SEQB_IRQHandler        [WEAK]
                LDR        R0, =ADC0_SEQB_DriverIRQHandler
                BX         R0
                ENDP

ADC0_THCMP_IRQHandler\
                PROC
                EXPORT     ADC0_THCMP_IRQHandler        [WEAK]
                LDR        R0, =ADC0_THCMP_DriverIRQHandler
                BX         R0
                ENDP

RTC_IRQHandler\
                PROC
                EXPORT     RTC_IRQHandler        [WEAK]
                LDR        R0, =RTC_DriverIRQHandler
                BX         R0
                ENDP

Reserved46_IRQHandler\
                PROC
                EXPORT     Reserved46_IRQHandler        [WEAK]
                LDR        R0, =Reserved46_DriverIRQHandler
                BX         R0
                ENDP

MAILBOX_IRQHandler\
                PROC
                EXPORT     MAILBOX_IRQHandler        [WEAK]
                LDR        R0, =MAILBOX_DriverIRQHandler
                BX         R0
                ENDP

GINT1_IRQHandler\
                PROC
                EXPORT     GINT1_IRQHandler        [WEAK]
                LDR        R0, =GINT1_DriverIRQHandler
                BX         R0
                ENDP

PIN_INT4_IRQHandler\
                PROC
                EXPORT     PIN_INT4_IRQHandler        [WEAK]
                LDR        R0, =PIN_INT4_DriverIRQHandler
                BX         R0
                ENDP

PIN_INT5_IRQHandler\
                PROC
                EXPORT     PIN_INT5_IRQHandler        [WEAK]
                LDR        R0, =PIN_INT5_DriverIRQHandler
                BX         R0
                ENDP

PIN_INT6_IRQHandler\
                PROC
                EXPORT     PIN_INT6_IRQHandler        [WEAK]
                LDR        R0, =PIN_INT6_DriverIRQHandler
                BX         R0
                ENDP

PIN_INT7_IRQHandler\
                PROC
                EXPORT     PIN_INT7_IRQHandler        [WEAK]
                LDR        R0, =PIN_INT7_DriverIRQHandler
                BX         R0
                ENDP

Reserved53_IRQHandler\
                PROC
                EXPORT     Reserved53_IRQHandler        [WEAK]
                LDR        R0, =Reserved53_DriverIRQHandler
                BX         R0
                ENDP

Reserved54_IRQHandler\
                PROC
                EXPORT     Reserved54_IRQHandler        [WEAK]
                LDR        R0, =Reserved54_DriverIRQHandler
                BX         R0
                ENDP

Reserved55_IRQHandler\
                PROC
                EXPORT     Reserved55_IRQHandler        [WEAK]
                LDR        R0, =Reserved55_DriverIRQHandler
                BX         R0
                ENDP

RIT_IRQHandler\
                PROC
                EXPORT     RIT_IRQHandler        [WEAK]
                LDR        R0, =RIT_DriverIRQHandler
                BX         R0
                ENDP

Default_Handler\
				PROC
                EXPORT     WDT_DriverIRQHandler               [WEAK]
                EXPORT     BOD_DriverIRQHandler               [WEAK]
                EXPORT     Reserved18_DriverIRQHandler        [WEAK]
                EXPORT     DMA0_DriverIRQHandler              [WEAK]
                EXPORT     GINT0_DriverIRQHandler             [WEAK]
                EXPORT     PIN_INT0_DriverIRQHandler          [WEAK]
                EXPORT     PIN_INT1_DriverIRQHandler          [WEAK]
                EXPORT     PIN_INT2_DriverIRQHandler          [WEAK]
                EXPORT     PIN_INT3_DriverIRQHandler          [WEAK]
                EXPORT     UTICK0_DriverIRQHandler            [WEAK]
                EXPORT     MRT0_DriverIRQHandler              [WEAK]
                EXPORT     CTIMER0_DriverIRQHandler           [WEAK]
                EXPORT     CTIMER1_DriverIRQHandler           [WEAK]
                EXPORT     CTIMER2_DriverIRQHandler           [WEAK]
                EXPORT     CTIMER3_DriverIRQHandler           [WEAK]
                EXPORT     CTIMER4_DriverIRQHandler           [WEAK]
                EXPORT     SCT0_DriverIRQHandler              [WEAK]
                EXPORT     USART0_DriverIRQHandler            [WEAK]
                EXPORT     USART1_DriverIRQHandler            [WEAK]
                EXPORT     USART2_DriverIRQHandler            [WEAK]
                EXPORT     USART3_DriverIRQHandler            [WEAK]
                EXPORT     I2C0_DriverIRQHandler              [WEAK]
                EXPORT     I2C1_DriverIRQHandler              [WEAK]
                EXPORT     I2C2_DriverIRQHandler              [WEAK]
                EXPORT     SPI0_DriverIRQHandler              [WEAK]
                EXPORT     SPI1_DriverIRQHandler              [WEAK]
                EXPORT     ADC0_SEQA_DriverIRQHandler         [WEAK]
                EXPORT     ADC0_SEQB_DriverIRQHandler         [WEAK]
                EXPORT     ADC0_THCMP_DriverIRQHandler        [WEAK]
                EXPORT     RTC_DriverIRQHandler               [WEAK]
                EXPORT     Reserved46_DriverIRQHandler        [WEAK]
                EXPORT     MAILBOX_DriverIRQHandler           [WEAK]
                EXPORT     GINT1_DriverIRQHandler             [WEAK]
                EXPORT     PIN_INT4_DriverIRQHandler          [WEAK]
                EXPORT     PIN_INT5_DriverIRQHandler          [WEAK]
                EXPORT     PIN_INT6_DriverIRQHandler          [WEAK]
                EXPORT     PIN_INT7_DriverIRQHandler          [WEAK]
                EXPORT     Reserved53_DriverIRQHandler        [WEAK]
                EXPORT     Reserved54_DriverIRQHandler        [WEAK]
                EXPORT     Reserved55_DriverIRQHandler        [WEAK]
                EXPORT     RIT_DriverIRQHandler               [WEAK]

WDT_DriverIRQHandler
BOD_DriverIRQHandler
Reserved18_DriverIRQHandler
DMA0_DriverIRQHandler
GINT0_DriverIRQHandler
PIN_INT0_DriverIRQHandler
PIN_INT1_DriverIRQHandler
PIN_INT2_DriverIRQHandler
PIN_INT3_DriverIRQHandler
UTICK0_DriverIRQHandler
MRT0_DriverIRQHandler
CTIMER0_DriverIRQHandler
CTIMER1_DriverIRQHandler
CTIMER2_DriverIRQHandler
CTIMER3_DriverIRQHandler
CTIMER4_DriverIRQHandler
SCT0_DriverIRQHandler
USART0_DriverIRQHandler
USART1_DriverIRQHandler
USART2_DriverIRQHandler
USART3_DriverIRQHandler
I2C0_DriverIRQHandler
I2C1_DriverIRQHandler
I2C2_DriverIRQHandler
SPI0_DriverIRQHandler
SPI1_DriverIRQHandler
ADC0_SEQA_DriverIRQHandler
ADC0_SEQB_DriverIRQHandler
ADC0_THCMP_DriverIRQHandler
RTC_DriverIRQHandler
Reserved46_DriverIRQHandler
MAILBOX_DriverIRQHandler
GINT1_DriverIRQHandler
PIN_INT4_DriverIRQHandler
PIN_INT5_DriverIRQHandler
PIN_INT6_DriverIRQHandler
PIN_INT7_DriverIRQHandler
Reserved53_DriverIRQHandler
Reserved54_DriverIRQHandler
Reserved55_DriverIRQHandler
RIT_DriverIRQHandler

                B       .

                ENDP
                
Reset_Handler
    LDR.W   R0, =0xE000ED88
    LDR     R1, [R0]
    ORR     R1, R1, #(0xF << 20)
    STR     R1, [R0]
    CPSID   I
	
    IMPORT  SystemInit
    IMPORT  __main
		
                IF      :LNOT::DEF:SLAVEBOOT
                ; Both the M0+ and M4 core come via this shared startup code,
                ; but the M0+ and M4 core have different vector tables.
                ; Determine if the core executing this code is the master or
                ; the slave and handle each core state individually.
shared_boot_entry
                LDR     r6, =rel_vals
                MOVS    r4, #0                          ; Flag for slave core (0)
                MOVS    r5, #1
		
                ; Determine which core (M0+ or M4) this code is running on
                ; r2 = (((*cpu_id) >> 4) & 0xFFF); (M4 core == 0xC24)
get_current_core_id
                LDR     r0, [r6, #0]
                LDR     r1, [r0]                        ; r1 = CPU ID status
                LSRS    r1, r1, #4                      ; Right justify 12 CPU ID bits
                LDRH    r2, [r6, #16]                   ; Mask for CPU ID bits
                ANDS    r2, r1, r2                      ; r2 = ARM COrtex CPU ID
                LDRH    r3, [r6, #18]                   ; Mask for CPU ID bits
                CMP     r3, r2                          ; Core ID matches M4 identifier
                BNE     get_master_status
                MOV     r4, r5                          ; Set flag for master core (1)

                ; Determine if M4 core is the master or slave
                ; r3 = ((*cpu_ctrl) & 1); (0 == m0+, 1 == M4)
get_master_status
                LDR     r0, [r6, #4]
                LDR     r3, [r0]                        ; r3 = SYSCON co-processor CPU control status
                ANDS    r3, r3, r5                      ; r3 = (Bit 0: 1 = M4 is master, 0 = M4 is slave)

                ; Select boot based on selected master core and core ID
select_boot
                EORS    r3, r3, r4                      ; r4 = (Bit 0: 0 = master, 1 = slave)
                BNE     slave_boot
                B       normal_boot

                ; Slave boot
slave_boot
                LDR     r0, [r6, #8]
                LDR     r2, [r0]                        ; r1 = SYSCON co-processor boot address
                CMP     r2, #0                          ; Slave boot address = 0 (not set up)?
                BEQ     cpu_sleep
                LDR     r0, [r6, #12]
                LDR     r1, [r0]                        ; r5 = SYSCON co-processor stack address
                MOV     sp, r1                          ; Update slave CPU stack pointer
                ; Be sure to update VTOR for the slave MCU to point to the
                ; slave vector table in boot memory
                BX      r2                              ; Jump to slave boot address

                ; Slave isn't yet setup for system boot from the master
                ; so sleep until the master sets it up and then reboots it
cpu_sleep
                MOV     sp, r5                          ; Will force exception if something happens
cpu_sleep_wfi
                WFI                                     ; Sleep forever until master reboots
                B       cpu_sleep_wfi
                ENDIF

                ; Normal boot for master/slave
normal_boot
                LDR     r0, =SystemInit
                BLX     r0
                LDR     r0, =__main
                BX      r0


    ALIGN

    IF      :DEF:__MICROLIB
                
    EXPORT  __initial_sp
    EXPORT  __heap_base
    EXPORT  __heap_limit
                
    ELSE
                
    IMPORT  __use_two_region_memory
    EXPORT  __user_initial_stackheap
                 
__user_initial_stackheap

    LDR     R0, =  Heap_Mem
    LDR     R1, =(Stack_Mem + Stack_Size)
    LDR     R2, = (Heap_Mem +  Heap_Size)
    LDR     R3, = Stack_Mem
    BX      LR

    ALIGN
    ENDIF


    END

