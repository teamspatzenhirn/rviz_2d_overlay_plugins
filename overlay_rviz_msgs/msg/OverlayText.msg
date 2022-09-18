uint8 ADD = 0
uint8 DELETE = 1

# constants for the horizontal and vertical alignment
uint8 LEFT = 0
uint8 RIGHT = 1
uint8 CENTER = 2
uint8 TOP = 3
uint8 BOTTOM = 4

uint8 action

int32 width
int32 height
int32 left
int32 top
int32 horizontal_distance # horizontal distance between the two anchor points
int32 vertical_distance # vertical distance between the two anchor points
uint8 horizontal_alignment # applied to overlay and parent frame
uint8 vertical_alignment # applied to overlay and parent frame

std_msgs/ColorRGBA bg_color

int32 line_width
float32 text_size
string font
std_msgs/ColorRGBA fg_color

string text
