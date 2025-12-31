/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Prism;

public class Color {
    public int red;
    public int green;
    public int blue;

    public Color(int red, int green, int blue)
    {
        this.red = Math.min(red, 255);
        this.green = Math.min(green, 255);
        this.blue = Math.min(blue, 255);
    }

    @Override
    public String toString()
    {
        return String.format("%d, %d, %d", red, green, blue);
    }

    public static final Color RED         = new Color(255, 0,   0);
    public static final Color ORANGE      = new Color(255, 165, 0);
    public static final Color YELLOW      = new Color(255, 255, 0);
    public static final Color OLIVE       = new Color(128, 128, 0);
    public static final Color GREEN       = new Color(0,   255, 0);
    public static final Color CYAN        = new Color(0,   255, 255);
    public static final Color BLUE        = new Color(0,   0,   255);
    public static final Color TEAL        = new Color(0,   128, 128);
    public static final Color MAGENTA     = new Color(255, 0,   255);
    public static final Color PURPLE      = new Color(128, 0,   128);
    public static final Color PINK        = new Color(255, 20,  128);
    public static final Color WHITE       = new Color(255, 255, 255);
    public static final Color TRANSPARENT = new Color(0,   0,   0);
}
