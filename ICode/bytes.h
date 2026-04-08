#pragma once
#include <stdint.h>
#include <string.h>

/* ---- 小端读（如仍需兼容旧报文） ---- */
static inline uint16_t rd_le16(const uint8_t* p){ return (uint16_t)p[0] | ((uint16_t)p[1]<<8); }
static inline uint32_t rd_le32(const uint8_t* p){ return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24); }
static inline void     wr_le16(uint8_t* p, uint16_t v){ p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static inline void     wr_le32(uint8_t* p, uint32_t v){ p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24); }

/* ---- 大端读写（本次改为主口径） ---- */
static inline uint16_t rd_be16(const uint8_t* p){ return ((uint16_t)p[0]<<8) | (uint16_t)p[1]; }
static inline uint32_t rd_be32(const uint8_t* p){ return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|(uint32_t)p[3]; }
static inline void     wr_be16(uint8_t* p, uint16_t v){ p[0]=(uint8_t)(v>>8); p[1]=(uint8_t)v; }
static inline void     wr_be32(uint8_t* p, uint32_t v){ p[0]=(uint8_t)(v>>24); p[1]=(uint8_t)(v>>16); p[2]=(uint8_t)(v>>8); p[3]=(uint8_t)v; }

/* 指针推进版（拼包更顺手） */
static inline void put_be_u16(uint8_t** pp, uint16_t v){ uint8_t* p=*pp; wr_be16(p,v); *pp=p+2; }
static inline void put_be_u32(uint8_t** pp, uint32_t v){ uint8_t* p=*pp; wr_be32(p,v); *pp=p+4; }
static inline void put_be_f32(uint8_t** pp, float f){ uint32_t u; memcpy(&u,&f,sizeof(u)); put_be_u32(pp,u); }
