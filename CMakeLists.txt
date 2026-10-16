cmake_minimum_required(VERSION 3.22)
project(MONO64 VERSION 0.3.0 LANGUAGES C CXX)

# Local JUCE copy is supported: cmake -S . -B build -DJUCE_DIR=C:/JUCE
# Otherwise, the first configuration downloads a pinned JUCE release.
if (DEFINED JUCE_DIR AND EXISTS "${JUCE_DIR}/CMakeLists.txt")
    add_subdirectory("${JUCE_DIR}" "${CMAKE_BINARY_DIR}/juce")
else()
    include(FetchContent)
    FetchContent_Declare(JUCE
        GIT_REPOSITORY https://github.com/juce-framework/JUCE.git
        GIT_TAG 9.0.4
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(JUCE)
endif()

juce_add_plugin(MONO64
    COMPANY_NAME "Independent Audio"
    PLUGIN_MANUFACTURER_CODE Mn64
    PLUGIN_CODE M064
    FORMATS VST3 Standalone
    PRODUCT_NAME "MONO64"
    IS_SYNTH TRUE
    NEEDS_MIDI_INPUT TRUE
    NEEDS_MIDI_OUTPUT FALSE
    IS_MIDI_EFFECT FALSE
    COPY_PLUGIN_AFTER_BUILD FALSE
)

target_sources(MONO64 PRIVATE Source/PluginProcessor.cpp Source/SynthEngine.h)
target_compile_features(MONO64 PRIVATE cxx_std_17)
target_compile_definitions(MONO64 PUBLIC
    JUCE_WEB_BROWSER=0
    JUCE_USE_CURL=0
    JUCE_VST3_CAN_REPLACE_VST2=0
)
target_link_libraries(MONO64 PRIVATE
    juce::juce_audio_utils
    juce::juce_audio_processors
    juce::juce_gui_extra
    juce::juce_dsp
    PUBLIC
    juce::juce_recommended_config_flags
    juce::juce_recommended_warning_flags
)
