#
# OpenWrt package for cmd-test
#

include $(TOPDIR)/rules.mk

PKG_NAME:=cmd-test
PKG_VERSION:=1.0
PKG_RELEASE:=1

PKG_BUILD_DIR:=$(BUILD_DIR)/$(PKG_NAME)

include $(INCLUDE_DIR)/package.mk

define Package/cmd-test
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=Command Test Client for IPCManager
  DEPENDS:=+libstdcpp
endef

define Package/cmd-test/description
  A small command line tool to test SPI IPC manager on OpenWrt.
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(TARGET_CXX) -o $(PKG_BUILD_DIR)/cmd-test $(PKG_BUILD_DIR)/cmd-test.cpp
endef

define Package/cmd-test/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/cmd-test $(1)/usr/bin/
endef

$(eval $(call BuildPackage,cmd-test))
