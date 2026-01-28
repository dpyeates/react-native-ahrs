Pod::Spec.new do |s|
  s.name         = "Eigen"
  s.version      = "5.0.0"
  s.summary      = "Eigen is a C++ template library for linear algebra."
  s.homepage     = "https://eigen.tuxfamily.org"
  s.license      = { :type => "MPL2" }
  s.author       = "Eigen Developers"
  
  s.platforms    = { :ios => "12.0" }
  s.source       = { :git => "https://gitlab.com/libeigen/eigen.git", :tag => "5.0.0" }
  
  s.public_header_files = "Eigen/**/*.h"
  s.source_files = "Eigen/**/*.h"
  s.header_mappings_dir = "."
  s.preserve_paths = "Eigen"
  
  s.pod_target_xcconfig = {
    'HEADER_SEARCH_PATHS' => '$(PODS_TARGET_SRCROOT)'
  }
end

