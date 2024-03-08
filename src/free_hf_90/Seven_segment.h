/* 
 Seven segment Look Up Tables
 */

/* 
 use an array of structs in a dict-type fashion to look up 7 segment display segments for each alphanumeric input character
 */
typedef struct { // frequency memory 
String alphanum; // corresponding alphanumeric character 
uint8_t segments; // segments which are lit 
} segdict; 

const uint8_t nalphas = 64;

const segdict lookup_segments[] = {
{"0", 246},
{"1", 34},
{"2", 218},
{"3", 122},
{"4", 46},
{"5", 124},
{"6", 252},
{"7", 50},
{"8", 254},
{"9", 126},
{"a", 190},
{"b", 236},
{"c", 212},
{"d", 234},
{"e", 220},
{"f", 156},
{"g", 126},
{"h", 174},
{"I", 34},
{"j", 98},
{"k", 142},
{"l", 196},
{"m", 0},
{"n", 182},
{"o", 246},
{"p", 158},
{"q", 62},
{"r", 148},
{"s", 124},
{"t", 204},
{"u", 230},
{"v", 224},
{"w", 0},
{"x", 0},
{"y", 110},
{"z", 218},
{"-", 8},
{"A", 190},
{"B", 236},
{"C", 212},
{"D", 234},
{"E", 220},
{"F", 156},
{"G", 126},
{"H", 174},
{"I", 34},
{"J", 98},
{"K", 142},
{"L", 196},
{"M", 0},
{"N", 182},
{"O", 246},
{"P", 158},
{"Q", 62},
{"R", 148},
{"S", 124},
{"T", 204},
{"U", 230},
{"V", 224},
{"W", 0},
{"X", 0},
{"Y", 110},
{"Z", 218},
{"^", 182},
};