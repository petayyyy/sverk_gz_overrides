import QtQuick 2.0

// The publisher has no visible controls. Gazebo GUI requires each GUI plugin
// to provide a QML resource even when it only supplies background behavior.
Item {
  visible: false
  width: 0
  height: 0
}
