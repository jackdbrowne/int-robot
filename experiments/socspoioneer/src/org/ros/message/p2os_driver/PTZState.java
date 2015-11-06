/* Auto-generated by genmsg_java.py for file /data/private/luser/workspace/usc-ros-pkg/p2os/p2os_driver/msg/PTZState.msg */

package org.ros.message.p2os_driver;

import java.nio.ByteBuffer;

public class PTZState extends org.ros.message.Message {

  public int pan;
  public int tilt;
  public int zoom;
  public boolean relative;

  public PTZState() {
  }

  public static java.lang.String __s_getDataType() { return "p2os_driver/PTZState"; }
  @Override  public java.lang.String getDataType() { return __s_getDataType(); }
  public static java.lang.String __s_getMD5Sum() { return "1f71ce2a42b32376ea869eb051358045"; }
  @Override  public java.lang.String getMD5Sum() { return __s_getMD5Sum(); }
  public static java.lang.String __s_getMessageDefinition() { return "int32 pan\n" +
"int32 tilt\n" +
"int32 zoom\n" +
"bool relative\n" +
"\n" +
""; }
  @Override  public java.lang.String getMessageDefinition() { return __s_getMessageDefinition(); }

  @Override
  public PTZState clone() {
    PTZState c = new PTZState();
    c.deserialize(serialize(0));
    return c;
  }

  @Override
  public void setTo(org.ros.message.Message m) {
    deserialize(m.serialize(0));
  }

  @Override
  public int serializationLength() {
    int __l = 0;
    __l += 4; // pan
    __l += 4; // tilt
    __l += 4; // zoom
    __l += 1; // relative
    return __l;
  }

  @Override
  public void serialize(ByteBuffer bb, int seq) {
    bb.putInt(pan);
    bb.putInt(tilt);
    bb.putInt(zoom);
    bb.put((byte)(relative ? 1 : 0));
  }

  @Override
  public void deserialize(ByteBuffer bb) {
    pan = bb.getInt();
    tilt = bb.getInt();
    zoom = bb.getInt();
    relative = bb.get() != 0 ? true : false;
  }

  @SuppressWarnings("all")
  public boolean equals(Object o) {
    if(!(o instanceof PTZState))
      return false;
    PTZState other = (PTZState) o;
    return
      pan == other.pan &&
      tilt == other.tilt &&
      zoom == other.zoom &&
      relative == other.relative &&
      true;
  }

  @SuppressWarnings("all")
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long tmp;
    result = prime * result + this.pan;
    result = prime * result + this.tilt;
    result = prime * result + this.zoom;
    result = prime * result + (this.relative ? 1231 : 1237);
    return result;
  }
} // class PTZState
