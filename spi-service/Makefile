include $(TOPDIR)/rules.mk

PKG_NAME:=spi-service
PKG_VERSION:=1.0
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

define Package/spi-service
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=SPI Service
  DEPENDS:=+libc +libstdcpp +libpthread
endef

define Build/Prepare
	# 將 src 下的 code 複製到 build_dir
	cp -r $(CURDIR)/src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	# 編譯 src 下的 Makefile
	$(MAKE) -C $(PKG_BUILD_DIR) \
		CXX="$(TARGET_CXX)" \
		CXXFLAGS="$(TARGET_CXXFLAGS) -std=c++17 -pthread"
endef

define Package/spi-service/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/spi-service $(1)/usr/bin/
endef

$(eval $(call BuildPackage,spi-service))
