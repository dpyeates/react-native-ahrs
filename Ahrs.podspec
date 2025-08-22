require "json"

package = JSON.parse(File.read(File.join(__dir__, "package.json")))

Pod::Spec.new do |s|
  s.name         = "Ahrs"
  s.version      = package["version"]
  s.summary      = package["description"]
  s.homepage     = package["homepage"]
  s.license      = package["license"]
  s.authors      = package["author"]

  s.platforms    = { :ios => min_ios_version_supported }
  s.source       = { :git => "https://github.com/dpyeates/react-native-ahrs.git", :tag => "#{s.version}" }

  s.source_files = "ios/**/*.{h,m,mm,swift}", "fusion/**/*.{hpp,cpp,c,h}"
  s.private_header_files = "ios/**/*.h", "fusion/**/*.{hpp,h}"

  s.frameworks = ['CoreMotion', 'QuartzCore']

  install_modules_dependencies(s)
end
