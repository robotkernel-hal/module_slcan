from conan import ConanFile

class MainProject(ConanFile):
    python_requires = "conan_template/[~5]@robotkernel/stable"
    python_requires_extend = "conan_template.RobotkernelConanFile"

    name = "module_slcan"
    description = ""
    exports_sources = ["*", "!.gitignore"] 
    
    def source(self):
        self.run(f"sed 's/AC_INIT(.*/AC_INIT([module_slcan], [{self.version}], [{self.author}])/' configure.ac.in > configure.ac")

    def requirements(self):
        self.requires("robotkernel/[~6]@robotkernel/unstable")
        self.requires("service_provider_memory_inspection/[~6]@robotkernel/unstable")
        self.requires("service_provider_canopen_protocol/[~6]@robotkernel/unstable")
        self.requires("service_provider_process_data_inspection/[~6]@robotkernel/unstable")
        self.requires("serialcan/0.3.0@3rdparty/unstable")

