#ifndef BITMANIP_H
#define BITMANIP_H
/**
 * @brief Macros for bit and bitfields manipulation
 */
///@{
// Bit macro (LSB=0)
#define BIT(N)                      (1UL<<(N))
// Bit manipulation using masks
#define BITSET(V,M)                 (V)|=(M)
#define BITCLEAR(V,M)               (V)&=~(M)
#define BITTOGGLE(V,M)              (V)^=(M)
// Bit manipulation using indexes
#define BITSETN(V,N)                 (V)|=BIT(N)
#define BITCLEARN(V,N)               (V)&=~BIT(N)
#define BITTOGGLEN(V,N)              (V)^=BIT(N)
// Bit fields
#define BITVALUE(V,N)               ((V)<<(N))
#define BITMASK(M,N)                ((BIT((M)-(N)+1)-1)<<(N))
// Bit fields using masks
#define BITFIELDGET(VAR,MASK)       ((VAR)&(MASK))
#define BITFIELDSET(VAR,MASK,VAL)   (VAR) = ((VAR)&~(MASK))|(VAL)
// Bit fields using indexes (M > N)
#define BITFIELDMNSET(VAR,M,N,VAL)  (VAR)=((VAR)&~(BITMASK((M),(N)))|BITVALUE((VAL),(N)))
#define BITFIELDMNGET(VAR,M,N)      ((VAR)&(BITMASK((M),(N)))>>(N))
//@}

/**
 * @brief Symbols for bits
 */
///@{
#define BIT0               1
#define BIT1               2
#define BIT2               4
#define BIT3               8
#define BIT4              16
#define BIT5              32
#define BIT6              64
#define BIT7             128
#define BIT8             256
#define BIT9             512
#define BIT10           1024
#define BIT11           2048
#define BIT12           4096
#define BIT13           8192
#define BIT14          16384
#define BIT15          32768
#define BIT16          65536
#define BIT17         131072
#define BIT18         262144
#define BIT19         524288
#define BIT20        1048576
#define BIT21        2097152
#define BIT22        4194304
#define BIT23        8388608
#define BIT24       16777216
#define BIT25       33554432
#define BIT26       67108864
#define BIT27      134217728
#define BIT28      268435456
#define BIT29      536870912
#define BIT30     1073741824
#define BIT31     2147483648

#endif // BITMANIP_H

