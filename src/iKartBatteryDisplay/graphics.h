#ifndef GRAPHICS_H
#define GRAPHICS_H

#include <gtkmm.h>
#define N_IMAGES 10

class GraphicsManager : public Gtk::Window
{
public:
  GraphicsManager();
  virtual ~GraphicsManager();
  void update_graphics(double voltage, double current, double charge);

protected:
  virtual void load_pixbufs();

  //signal handlers:
  virtual bool on_drawingarea_expose(GdkEventExpose *event);


  //Member widgets:
  Glib::RefPtr<Gdk::Pixbuf> m_refPixbuf;
  Glib::RefPtr<Gdk::Pixbuf> m_refPixbuf_Background;
  Glib::RefPtr<Gdk::Pixbuf> m_refPixbuf_Numbers;
  Glib::RefPtr<Gdk::Pixbuf> m_refPixbuf_Blocks;
  Glib::RefPtr<Gdk::Pixbuf> m_images[N_IMAGES];
  Gtk::DrawingArea m_DrawingArea;

  guint m_back_width, m_back_height;
  gint m_frame_num;
};

#endif
