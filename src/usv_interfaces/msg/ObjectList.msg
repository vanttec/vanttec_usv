# Object List msg
#
# header.stamp:    momento de la detección
# header.frame_id: frame de x, y de cada objeto
#   - "ned"        -> NED local del mundo (mismo datum que usv/loc)
#   - otro (ej. "base_link") -> relativo al bote: x hacia la proa, y hacia estribor [m]

std_msgs/Header header
Object[] obj_list
