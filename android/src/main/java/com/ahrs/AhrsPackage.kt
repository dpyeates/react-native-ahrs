package com.ahrs

import com.facebook.react.BaseReactPackage
import com.facebook.react.bridge.NativeModule
import com.facebook.react.bridge.ReactApplicationContext
import com.facebook.react.module.model.ReactModuleInfo
import com.facebook.react.module.model.ReactModuleInfoProvider
import com.facebook.react.turbomodule.core.interfaces.TurboModule

class AhrsPackage : BaseReactPackage() {
  override fun getModule(name: String, reactContext: ReactApplicationContext): NativeModule? {
    return if (name == AhrsModule.NAME) {
      AhrsModule(reactContext)
    } else {
      null
    }
  }

  override fun getReactModuleInfoProvider(): ReactModuleInfoProvider {
    return ReactModuleInfoProvider {
        mapOf(
            AhrsModule.NAME to ReactModuleInfo(
                AhrsModule.NAME,
                AhrsModule.NAME,
                false, // canOverrideExistingModule
                false, // needsEagerInit
                true,  // hasConstants
                true   // isTurboModule
            )
        )
    }
  }
}
