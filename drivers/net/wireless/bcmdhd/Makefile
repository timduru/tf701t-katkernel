# bcm43xx
DHDCFLAGS = -Wall -Wstrict-prototypes -Dlinux -DBCMDRIVER                     \
	-DDHD_FIRSTREAD=128 -DMAX_HDR_READ=128                                \
	-DPROP_TXSTATUS_VSDB -DDHDTCPACK_SUPPRESS                             \
	-DBCMDONGLEHOST -DUNRELEASEDCHIP -DBCMDMA32 -DBCMFILEIMAGE            \
	-DDHDTHREAD -DDHD_DEBUG -DSDTEST -DBDC -DTOE                          \
	-DDHD_BCMEVENTS -DSHOW_EVENTS -DPROP_TXSTATUS -DBCMDBG                \
	-DCUSTOMER_HW2 -DOOB_INTR_ONLY -DHW_OOB                               \
	-DMMC_SDIO_ABORT -DBCMSDIO -DBCMLXSDMMC -DBCMPLATFORM_BUS -DWLP2P     \
	-DWIFI_ACT_FRAME -DARP_OFFLOAD_SUPPORT                                \
	-DKEEP_ALIVE -DGET_CUSTOM_MAC_ENABLE -DPKT_FILTER_SUPPORT             \
	-DEMBEDDED_PLATFORM -DENABLE_INSMOD_NO_FW_LOAD -DPNO_SUPPORT          \
	-DDHD_USE_IDLECOUNT -DROAM_ENABLE -DVSDB                              \
	-DWL_CFG80211_VSDB_PRIORITIZE_SCAN_REQUEST                            \
	-DESCAN_RESULT_PATCH -DSDIO_CRC_ERROR_FIX -DBLACKLIST_PKT_FILTER      \
	-DDHD_DONOT_FORWARD_BCMEVENT_AS_NETWORK_PKT                           \
	-DCUSTOM_SDIO_F2_BLKSIZE=128 -DWL_SDO -DWLTDLS                        \
	-DWL_SUPPORT_BACKPORTED_KPATCHES -DDYNAMIC_DTIM_SKIP \
	-Idrivers/net/wireless/bcm43xx -Idrivers/net/wireless/bcm43xx/include

DHDOFILES = aiutils.o bcmsdh_sdmmc_linux.o dhd_linux.o siutils.o bcmutils.o   \
	dhd_linux_sched.o dhd_sdio.o bcmevent.o hndpmu.o dhd_pno.o            \
	bcmsdh.o dhd_cdc.o bcmsdh_linux.o dhd_common.o dhd_wlfc.o dhd_ip.o linux_osl.o \
	bcmwifi_channels.o bcmsdh_sdmmc.o dhd_custom_gpio.o sbutils.o wldev_common.o wl_android.o

ifeq ($(CONFIG_BCM43XX_WIFI_CONTROL_FUNC),y)
DHDCFLAGS += -DCONFIG_WIFI_CONTROL_FUNC
else
DHDCFLAGS += -DCUSTOM_OOB_GPIO_NUM=2
endif

ifeq ($(CONFIG_BCM43XX_EDP_SUPPORT),y)
DHDCFLAGS += -DWIFIEDP
endif

obj-$(CONFIG_BCM43XX) += bcmdhd.o
bcmdhd-objs += $(DHDOFILES)
ifneq ($(CONFIG_WIRELESS_EXT),)
bcmdhd-objs += wl_iw.o
DHDCFLAGS += -DSOFTAP -DWL_WIRELESS_EXT -DUSE_IW
endif
ifneq ($(CONFIG_CFG80211),)
bcmdhd-objs += wl_cfg80211.o wl_cfgp2p.o wl_linux_mon.o dhd_cfg80211.o
DHDCFLAGS += -DWL_CFG80211 -DWL_CFG80211_STA_EVENT
DHDCFLAGS += -DWL_ENABLE_P2P_IF -DWL_IFACE_COMB_NUM_CHANNELS
endif

DHDCFLAGS += -DCUSTOM_ROAM_TRIGGER_SETTING=-85
DHDCFLAGS += -DCUSTOM_ROAM_DELTA_SETTING=15
DHDCFLAGS += -DCUSTOM_KEEP_ALIVE_SETTING=10000
DHDCFLAGS += -DCUSTOM_PNO_EVENT_LOCK_xTIME=7
DHDCFLAGS += -DDHCP_SCAN_SUPPRESS

## Set dhd_dpd_thread priority to MAX to avoid starvation
DHDCFLAGS += -DCUSTOM_DPC_PRIO_SETTING=99

ifneq ($(CONFIG_DHD_USE_SCHED_SCAN),)
DHDCFLAGS += -DWL_SCHED_SCAN
endif

ifneq ($(CONFIG_BCM4334X),)
DHDCFLAGS += -DBCM4334X_MCC_ENABLE # Enable MCC
DHDCFLAGS += -DMIRACAST_MCHAN_ALGO=0
DHDCFLAGS += -DMIRACAST_AMPDU_BA_WSIZE=16
DHDCFLAGS += -DMIRACAST_MCHAN_BW=40
endif

ccflags-y = -I$(src)
ccflags-y += -I$(src)/include

EXTRA_CFLAGS = $(DHDCFLAGS)
ifeq ($(CONFIG_BCM43XX),m)
EXTRA_LDFLAGS += --strip-debug
endif
