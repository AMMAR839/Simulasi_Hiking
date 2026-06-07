import QtQuick 2.12

Rectangle {
  width: 320
  height: 78
  radius: 4
  color: "#20242ccc"
  border.color: "#6d7686"
  border.width: 1

  Text {
    anchors.fill: parent
    anchors.margins: 8
    color: "#f4f6f8"
    font.pixelSize: 12
    lineHeight: 1.15
    text: "Hiking LoRa keyboard active\n1-9/0 follow hiker, WASD move, B SOS, R auto\nQ follow, E overview, IJKL/UO adjust overview"
    verticalAlignment: Text.AlignVCenter
  }
}
