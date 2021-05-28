TEMPLATE        = app
TARGET          = D:\Ricerche\Tirocinio\cinolib\examples\01_base_app_trimesh_demo
QT             += core opengl
CONFIG         += c++11 release
CONFIG         -= app_bundle
INCLUDEPATH    += D:\Ricerche\Tirocinio\cinolib\examples\include
INCLUDEPATH    += D:\Ricerche\Tirocinio\cinolib\examples\external/eigen
LIBS += -LD:\Qt\Tools\mingw810_64\x86_64-w64-mingw32\lib/ -lglu32 -lopengl32
DEFINES        += CINOLIB_USES_OPENGL
DEFINES        += CINOLIB_USES_QT
QMAKE_CXXFLAGS += -Wno-deprecated-declarations # gluQuadric gluSphere and gluCylinde are deprecated in macOS 10.9
DATA_PATH       = \\\"D:\Ricerche\Tirocinio\cinolib\examples\data/\\\"
DEFINES        += DATA_PATH=$$DATA_PATH
SOURCES        += main.cpp

# just for Linux
unix:!macx {
DEFINES += GL_GLEXT_PROTOTYPES
LIBS    += -lGLU
}
