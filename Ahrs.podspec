require "json"

package = JSON.parse(File.read(File.join(__dir__, "package.json")))

# React Native helper - load if available
if defined?(install_modules_dependencies)
  # Use the helper
else
  # Define a no-op version for standalone validation
  def install_modules_dependencies(s)
    # This will be overridden by React Native's podhelper if present
  end
end

Pod::Spec.new do |s|
  s.name         = "Ahrs"
  s.version      = package["version"]
  s.summary      = package["description"]
  s.homepage     = package["homepage"]
  s.license      = package["license"]
  s.authors      = package["author"]

  # Minimum iOS version: 15.1
  s.platforms    = { :ios => "15.1" }
  s.source       = { :git => "https://github.com/dpyeates/react-native-ahrs.git", :tag => "#{s.version}" }

  s.source_files = "ios/**/*.{h,m,mm,swift}",
                   "fusionml/src/uNavINS.{h,cpp}",
                   "fusionml/src/AltitudeCalculator.{h,cpp}",
                   "fusionml/src/FlightPhaseDetector.{h,cpp}",
                   "fusionml/src/JsonRecorder.{h,cpp}"
  
  s.private_header_files = "ios/**/*.h",
                           "fusionml/src/uNavINS.h",
                           "fusionml/src/AltitudeCalculator.h",
                           "fusionml/src/FlightPhaseDetector.h",
                           "fusionml/src/JsonRecorder.h"
  
  # Preserve Eigen directory structure (files without extensions need to be preserved)
  s.preserve_paths = "fusionml/src/Eigen/**/*"
  
  # Exclude Eigen license file
  s.exclude_files = "fusionml/src/Eigen/COPYING.MPL2"

  s.pod_target_xcconfig = {
    'HEADER_SEARCH_PATHS' => '$(inherited) $(PODS_TARGET_SRCROOT)/fusionml/src',
    'CLANG_CXX_LANGUAGE_STANDARD' => 'c++14',
    'CLANG_CXX_LIBRARY' => 'libc++',
    'OTHER_CFLAGS' => '-DEIGEN_MPL2_ONLY -DEIGEN_NO_DEBUG -DNDEBUG -DEIGEN_DONT_VECTORIZE -DEIGEN_DONT_ALIGN -D_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_NONE',
    'OTHER_CPLUSPLUSFLAGS' => '-DEIGEN_MPL2_ONLY -DEIGEN_NO_DEBUG -DNDEBUG -DEIGEN_DONT_VECTORIZE -DEIGEN_DONT_ALIGN -D_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_NONE',
    'GCC_PREPROCESSOR_DEFINITIONS' => '$(inherited) EIGEN_MPL2_ONLY=1 EIGEN_NO_DEBUG=1 NDEBUG=1 EIGEN_DONT_VECTORIZE=1 EIGEN_DONT_ALIGN=1 _LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_NONE'
  }

  s.frameworks = ['CoreMotion', 'CoreLocation']

  s.libraries = ['z', 'm', 'c++']
  
  install_modules_dependencies(s)
end
