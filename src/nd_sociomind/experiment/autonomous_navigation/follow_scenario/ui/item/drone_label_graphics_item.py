from PySide6.QtGui import QBrush, QColor, QFont
from PySide6.QtWidgets import QGraphicsItem, QGraphicsSimpleTextItem


class DroneLabelGraphicsItem(QGraphicsSimpleTextItem):
    def __init__(self, text: str, text_color: QColor):
        super().__init__(text)
        font = QFont()
        font.setPointSizeF(8.5)
        font.setWeight(QFont.Weight.DemiBold)
        self.setFont(font)
        self.setBrush(QBrush(text_color))
        self.setZValue(30.0)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations, True)

    def set_anchor_position(self, x_pixels: float, y_pixels: float) -> None:
        text_bounds = self.boundingRect()
        self.setPos(x_pixels - text_bounds.width() * 0.5, y_pixels + 18.0)
