ccflags-y +=-I$(MTK_PATH_PLATFORM)/drivers/power/

ifeq ($(MTK_BQ24196_SUPPORT),yes) 
  obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_bq24196.o bq24196.o tbl_helper_func.o
else
  ifeq ($(MTK_BQ24297_SUPPORT),yes)
    obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_bq24297.o bq24297.o tbl_helper_func.o
  else
    ifeq ($(MTK_BQ24296_SUPPORT),yes)
      obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_bq24296.o bq24296.o tbl_helper_func.o
    endif
  endif
endif