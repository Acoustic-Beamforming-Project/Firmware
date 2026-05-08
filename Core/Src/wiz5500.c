/* wiz5500.c — W5500 UDP driver, SPI1, Optimized for high-throughput
 *
 * ═══════════════════════════════════════════════════════════════
 * OPTIMIZATIONS:
 *   1. Single-phase DMA for payload.
 *   2. Sn_CR=SEND issued via fast blocking SPI in DMA callback.
 *   3. No blocking Sn_IR_SENDOK poll in the hot path.
 *   4. Circular buffer Sn_TX_WR management allows overlapping
 *      processing and transmission.
 * ═══════════════════════════════════════════════════════════════
 */

#include "wiz5500.h"
#include "w5500.h"
#include "wizchip_conf.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;

/* Define the global WIZCHIP handle required by the ioLibrary */
WIZCHIPHandle_t WIZCHIP;

static WIZ5500_Config    _cfg;
static volatile uint32_t _dropped_packets = 0u;

typedef enum {
    PHASE_IDLE    = 0,
    PHASE_BUSY    = 1
} TxPhase;
static volatile TxPhase _phase = PHASE_IDLE;

/* ── Shadow TX write pointer ─────────────────────────────────────────── */
static uint16_t _tx_wr = 0u;

/* ── Pre-built SPI header ───────────────────────────────────────────── */
static uint8_t _tx_header[3] = {0, 0, W5500_CTRL_S0TX_W};

/* ── ioLibrary callbacks ────────────────────────────────────────────── */
static void    _cs_sel(void)  { HAL_GPIO_WritePin(ETH_CS_PORT,ETH_CS_PIN,GPIO_PIN_RESET); }
static void    _cs_dsel(void) { HAL_GPIO_WritePin(ETH_CS_PORT,ETH_CS_PIN,GPIO_PIN_SET);   }
static void    _wb(uint8_t b)        { HAL_SPI_Transmit(&hspi1,&b,1,10); }
static uint8_t _rb(void)             { uint8_t b=0; HAL_SPI_Receive(&hspi1,&b,1,10); return b; }
static void    _wburst(uint8_t *p,uint16_t n){ HAL_SPI_Transmit(&hspi1,p,n,50); }
static void    _rburst(uint8_t *p,uint16_t n){ HAL_SPI_Receive (&hspi1,p,n,50); }
static void    _ci(void) { __disable_irq(); }
static void    _ce(void) { __enable_irq();  }

static void _wizchip_init(void)
{
    memset(&WIZCHIP, 0, sizeof(WIZCHIPHandle_t));
    WIZCHIP.CS._select         = _cs_sel;
    WIZCHIP.CS._deselect       = _cs_dsel;
    WIZCHIP.IF.SPI._write_byte  = _wb;
    WIZCHIP.IF.SPI._read_byte   = _rb;
    WIZCHIP.IF.SPI._write_burst = _wburst;
    WIZCHIP.IF.SPI._read_burst  = _rburst;
    WIZCHIP.CRIS._enter        = _ci;
    WIZCHIP.CRIS._exit         = _ce;
}

static void _hw_reset(void)
{
    ETH_RST_ASSERT(); HAL_Delay(1);
    ETH_RST_DEASSERT(); HAL_Delay(55);
    setMR(MR_RST); HAL_Delay(5);
}

static void _net_config(void)
{
    setSHAR(_cfg.mac); setGAR(_cfg.gateway);
    setSUBR(_cfg.subnet); setSIPR(_cfg.ip);
    setRTR(2000); setRCR(8);
}

/* ═══════════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════════ */

void WIZ5500_Init(const WIZ5500_Config *cfg)
{
    if (!cfg) return;
    _cfg=*cfg;
    _dropped_packets=0; _phase=PHASE_IDLE; _tx_wr=0;
    _wizchip_init();
    _hw_reset();
    _net_config();
}

WIZ5500_Status WIZ5500_SetupSocket0(void)
{
    uint8_t sr; uint32_t deadline;
    setSn_CR(0,Sn_CR_CLOSE);
    setSn_TXBUF_SIZE(0,16); setSn_RXBUF_SIZE(0,0);
    setSn_MR(0,Sn_MR_UDP);
    { uint8_t bc[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; setSn_DHAR(0,bc); }
    setSn_DIPR(0,_cfg.dest_ip); setSn_DPORT(0,_cfg.dest_port);
    setSn_PORT(0,_cfg.src_port); setSn_CR(0,Sn_CR_OPEN);
    deadline=HAL_GetTick()+10;
    do { sr=getSn_SR(0); } while(sr!=SOCK_UDP && HAL_GetTick()<deadline);
    _tx_wr=getSn_TX_WR(0);
    return (sr==SOCK_UDP)?WIZ5500_OK:WIZ5500_ERR_SOCKET;
}

WIZ5500_Status WIZ5500_SendBatch(const uint8_t *src, uint16_t len)
{
    if (!src || len > 2048) return WIZ5500_ERR_PARAM;

    /* If SPI DMA is still busy, we can't start a new transfer. */
    if (_phase == PHASE_BUSY) {
        _dropped_packets++;
        return WIZ5500_ERR_BUSY;
    }

    /* Check W5500 free space (16KB total) */
    if (getSn_TX_FSR(0) < len) {
        _dropped_packets++;
        return WIZ5500_ERR_NO_SPACE;
    }

    _phase = PHASE_BUSY;

    /* 1. Write payload to W5500 TX buffer using DMA */
    uint16_t offset = _tx_wr & 0x3FFFu;
    _tx_header[0] = (uint8_t)(offset >> 8);
    _tx_header[1] = (uint8_t)(offset & 0xFF);

    ETH_CS_ASSERT();
    /* Fast header transmit (blocking, ~1us) */
    HAL_SPI_Transmit(&hspi1, _tx_header, 3, 2);
    /* Payload transmit (DMA) */
    HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)src, len);

    /* 2. Advance shadow write pointer */
    _tx_wr += len;

    return WIZ5500_OK;
}

void WIZ5500_OnDMATxComplete(void)
{
    ETH_CS_DEASSERT();

    /* 3. Update W5500 TX_WR register and issue SEND command.
     * These are register writes, very fast (~2us total). */
    setSn_TX_WR(0, _tx_wr);
    setSn_CR(0, Sn_CR_SEND);

    _phase = PHASE_IDLE;
}

uint32_t WIZ5500_GetDroppedPackets(void) { return _dropped_packets; }
uint8_t  WIZ5500_IsTxBusy(void)         { return (_phase != PHASE_IDLE); }
