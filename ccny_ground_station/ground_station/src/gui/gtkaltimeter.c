#include "ground_station/gui/gtkaltimeter.h"

#define GTK_ALTIMETER_GET_PRIVATE(obj) (G_TYPE_INSTANCE_GET_PRIVATE ((obj), GTK_ALTIMETER_TYPE, GtkAltimeterPrivate))

G_DEFINE_TYPE (GtkAltimeter, gtk_altimeter, GTK_TYPE_DRAWING_AREA);

static gboolean gtk_altimeter_expose (GtkWidget *clock, GdkEventExpose *event);
static gboolean gtk_altimeter_update (gpointer data);

typedef struct _GtkAltimeterPrivate GtkAltimeterPrivate;

struct _GtkAltimeterPrivate
{
	gint unit_value;
	gboolean unit_is_feet;
	gboolean color_mode_inv;
	gdouble altitude;
};

static void
gtk_altimeter_class_init (GtkAltimeterClass *class)
{
	GObjectClass *obj_class;
	GtkWidgetClass *widget_class;

	obj_class = G_OBJECT_CLASS (class);
	widget_class = GTK_WIDGET_CLASS (class);

	widget_class->expose_event = gtk_altimeter_expose;

	g_type_class_add_private (obj_class, sizeof (GtkAltimeterPrivate));
}

static void
gtk_altimeter_init (GtkAltimeter *alt)
{
	GtkAltimeterPrivate *priv;
	priv = GTK_ALTIMETER_GET_PRIVATE (alt);
	priv->unit_is_feet=TRUE;
	priv->unit_value=100;
	priv->color_mode_inv=FALSE;
	priv->altitude = 0.;
	
	gtk_altimeter_update(alt);
	
	g_timeout_add(100,gtk_altimeter_update,alt);
}

static void
draw (GtkWidget *alt, cairo_t *cr)
{
	GtkAltimeterPrivate *priv;
	double x, y, rec_x0, rec_y0, rec_width, rec_height, rec_degrees;
	double rec_aspect, rec_corner_radius, rec_radius, radius;
	char str[5];
	int i;
	double back_r=0.1,back_g=0.1,back_b=0.1;
	double back_r_inv=0.7,back_g_inv=0.7,back_b_inv=0.7;
	int factor=1;

	priv = GTK_ALTIMETER_GET_PRIVATE (alt);
	
	x = alt->allocation.x + alt->allocation.width / 2;
	y = alt->allocation.y + alt->allocation.height / 2;
	radius = MIN (alt->allocation.width / 2,
		      alt->allocation.height / 2) - 5;

	rec_x0=x-radius;    
   rec_y0=y-radius;
   rec_width=radius*2;
   rec_height=radius*2;
   rec_aspect=1.0;
   rec_corner_radius=rec_height/8.0;

	rec_radius=rec_corner_radius/rec_aspect;
	rec_degrees=M_PI/180.0;

	// Altimeter base
	cairo_new_sub_path (cr);
	cairo_arc (cr, rec_x0+rec_width-rec_radius, rec_y0+rec_radius,
					rec_radius, -90*rec_degrees, 0*rec_degrees);
	cairo_arc (cr, rec_x0+rec_width-rec_radius, rec_y0+rec_height-rec_radius,
					rec_radius, 0*rec_degrees, 90*rec_degrees);
	cairo_arc (cr, rec_x0+rec_radius, rec_y0+rec_height-rec_radius,
					rec_radius, 90*rec_degrees, 180*rec_degrees);
	cairo_arc (cr, rec_x0+rec_radius, rec_y0+rec_radius, rec_radius,
					180*rec_degrees, 270*rec_degrees);
	cairo_close_path (cr);

	if(!priv->color_mode_inv) cairo_set_source_rgb (cr, back_r, back_g, back_b);
	else cairo_set_source_rgb (cr, back_r_inv, back_g_inv, back_b_inv);
	cairo_fill_preserve (cr);
	cairo_stroke (cr);

	cairo_arc (cr, x, y, radius, 0, 2 * M_PI);
	if(!priv->color_mode_inv) cairo_set_source_rgb (cr, 0., 0., 0.);
	else cairo_set_source_rgb (cr, 1., 1., 1.);
	cairo_fill_preserve (cr);
	cairo_stroke (cr);
	
	cairo_arc (cr, x, y, radius-0.04*radius, 0, 2 * M_PI);
	if(!priv->color_mode_inv) cairo_set_source_rgb (cr, 0.6, 0.5, 0.5);
	else cairo_set_source_rgb (cr, 1-0.6, 1-0.5, 1-0.5);
	cairo_stroke (cr);

	cairo_set_line_width (cr,0.01*radius);
	radius=radius-0.1*radius;
	cairo_arc (cr, x, y, radius, 0, 2 * M_PI);
	if(!priv->color_mode_inv) cairo_set_source_rgb (cr, back_r, back_g, back_b);
	else cairo_set_source_rgb (cr, back_r_inv, back_g_inv, back_b_inv);
	cairo_fill_preserve (cr);
	cairo_stroke (cr);

	cairo_arc (cr, x, y, radius, 0, 2 * M_PI);
	if(!priv->color_mode_inv) cairo_set_source_rgb (cr, 0.6, 0.6, 0.6);
	else cairo_set_source_rgb (cr, 1-0.6, 1-0.6, 1-0.6);
	cairo_fill_preserve (cr);
	cairo_stroke (cr);
	
	cairo_arc (cr, x, y, radius-0.07*radius, 0, 2 * M_PI);
	if(!priv->color_mode_inv) cairo_set_source_rgb (cr, back_r, back_g, back_b);
	else cairo_set_source_rgb (cr, back_r_inv, back_g_inv, back_b_inv);
	cairo_fill_preserve (cr);
	cairo_stroke (cr);
	
	// fun
	cairo_arc (cr, x, y, radius-0.45*radius,M_PI/3,2*M_PI/3);
	cairo_move_to (cr,x+0.27*radius,y+0.4764*radius);
	cairo_arc (cr, x, y, radius-0.8*radius,M_PI/3,2*M_PI/3);
	cairo_line_to (cr,x-0.27*radius,y+0.4764*radius);
	if(!priv->color_mode_inv) cairo_set_source_rgb (cr, 0.9, 0.9, 0.9);
	else cairo_set_source_rgb (cr, 1-0.9, 1-0.9, 1-0.9);
	cairo_fill_preserve (cr);	
	cairo_stroke (cr);
	
	cairo_set_line_width (cr, 0.03*radius);
	cairo_move_to (cr,x-0.7*radius,y);	
	cairo_line_to (cr,x-0.1*radius,y+0.6*radius);
	cairo_move_to (cr,x-0.6*radius,y);	
	cairo_line_to (cr,x+0*radius,y+0.6*radius);
	cairo_move_to (cr,x-0.5*radius,y);	
	cairo_line_to (cr,x+0.1*radius,y+0.6*radius);
	cairo_move_to (cr,x-0.4*radius,y);	
	cairo_line_to (cr,x+0.2*radius,y+0.6*radius);
	cairo_move_to (cr,x-0.3*radius,y);	
	cairo_line_to (cr,x+0.3*radius,y+0.6*radius);
	cairo_move_to (cr,x-0.2*radius,y);	
	cairo_line_to (cr,x+0.4*radius,y+0.6*radius);
	cairo_move_to (cr,x-0.1*radius,y);	
	cairo_line_to (cr,x+0.5*radius,y+0.6*radius);
	if(!priv->color_mode_inv) cairo_set_source_rgb (cr, back_r, back_g, back_b);
	else cairo_set_source_rgb (cr, back_r_inv, back_g_inv, back_b_inv);
	cairo_stroke (cr);	
	
	// Altimeter ticks 
	if(!priv->color_mode_inv) cairo_set_source_rgb(cr, 1, 1, 1);
	else cairo_set_source_rgb (cr, 0., 0., 0.);
	for (i = 0; i < 50; i++)
	{
		int inset;
		cairo_save (cr);
		
		if (i % 5 == 0) inset = 0.12 * radius;
		else
		{
			inset = 0.06*radius;
			cairo_set_line_width (cr, 0.5 *cairo_get_line_width (cr));
		}
		
		cairo_move_to (cr,x+(radius-inset)*cos(M_PI/2+i*M_PI/25),
								y+(radius-inset)*sin(M_PI/2+i*M_PI/25));
		cairo_line_to (cr,x+radius*cos(M_PI/2+i*M_PI/25),
								y+radius*sin(M_PI/2+i*M_PI/25));
		cairo_stroke (cr);
		cairo_restore (cr);
	}
	
	// "Altimeter" drawing
	cairo_select_font_face (cr, "Sans", CAIRO_FONT_SLANT_OBLIQUE,
                               CAIRO_FONT_WEIGHT_NORMAL);
   cairo_set_font_size (cr, 0.1*radius);
	cairo_move_to (cr,x-0.23*radius,y-0.12*radius);
	cairo_show_text (cr,"Altimeter");	
	cairo_stroke (cr);
	
		
	cairo_select_font_face (cr, "Sans", CAIRO_FONT_SLANT_NORMAL,
                               CAIRO_FONT_WEIGHT_NORMAL);
	if (priv->unit_is_feet){
		// drawing unit : FEET
		cairo_save(cr);
		cairo_set_font_size (cr, 0.07*radius);
		cairo_move_to (cr,x+0.18*radius,y-0.83*radius);
		cairo_rotate (cr, M_PI / 10);
		cairo_show_text (cr,"FEET");	
		cairo_stroke (cr);
		cairo_restore(cr);
	}
	else
	{
		// drawing unit : METER
		cairo_save(cr);
		cairo_set_font_size (cr, 0.07*radius);
		cairo_move_to (cr,x+0.145*radius,y-0.85*radius);
		cairo_rotate (cr, M_PI / 10);
		cairo_show_text (cr,"METER");	
		cairo_stroke (cr);
		cairo_restore(cr);
	}
	switch(priv->unit_value)
	{
		case 1:
			cairo_save(cr);
			cairo_set_font_size (cr, 0.07*radius);
			cairo_move_to (cr,x-0.29*radius,y-0.81*radius);
			cairo_rotate (cr, -M_PI / 10);
			sprintf(str,"%d",priv->unit_value);
			cairo_show_text (cr,str);	
			cairo_stroke (cr);
			cairo_restore(cr);
			factor=100;
			break;
		case 10:
			cairo_save(cr);
			cairo_set_font_size (cr, 0.07*radius);
			cairo_move_to (cr,x-0.31*radius,y-0.8*radius);
			cairo_rotate (cr, -M_PI / 10);
			sprintf(str,"%d",priv->unit_value);
			cairo_show_text (cr,str);	
			cairo_stroke (cr);
			cairo_restore(cr);
			factor=10;
			break;
		case 100:
			cairo_save(cr);
			cairo_set_font_size (cr, 0.07*radius);
			cairo_move_to (cr,x-0.33*radius,y-0.78*radius);
			cairo_rotate (cr, -M_PI / 10);
			sprintf(str,"%d",priv->unit_value);
			cairo_show_text (cr,str);	
			cairo_stroke (cr);
			cairo_restore(cr);
			factor=1;
			break;
	}

	// Number drawing
	for (i = 0; i < 10; i++)
	{
		int inset;
		cairo_select_font_face (cr, "Sans", CAIRO_FONT_SLANT_NORMAL,
                               CAIRO_FONT_WEIGHT_NORMAL);
      cairo_set_font_size (cr, 0.20*radius);
		inset = 0.225*radius;
		cairo_move_to (cr,x-0.065*radius+(radius-inset)*cos(M_PI/2+i*M_PI/5+M_PI),
								y+0.07*radius+(radius-inset)*sin(M_PI/2+i*M_PI/5+M_PI));
		sprintf(str,"%d",i);
		cairo_show_text (cr, str);	
		cairo_stroke (cr);
	}
		
	//digital aff
	cairo_set_line_width (cr,1);
	cairo_rectangle(cr,x-0.145*radius,y-0.29*radius-0.165*radius,0.145*radius,0.18*radius);
	cairo_rectangle(cr,x-0.29*radius,y-0.29*radius-0.165*radius,0.145*radius,0.18*radius);
	cairo_rectangle(cr,x-0.435*radius,y-0.29*radius-0.165*radius,0.145*radius,0.18*radius);
	cairo_rectangle(cr,x+0*radius,y-0.29*radius-0.165*radius,0.145*radius,0.18*radius);
	cairo_rectangle(cr,x+0.145*radius,y-0.29*radius-0.165*radius,0.145*radius,0.18*radius);
	cairo_rectangle(cr,x+0.29*radius,y-0.29*radius-0.165*radius,0.145*radius,0.18*radius);
	if(!priv->color_mode_inv) cairo_set_source_rgb (cr, 0, 0, 0);
	else cairo_set_source_rgb (cr, 1, 1, 1);
	cairo_fill_preserve (cr);
	if(!priv->color_mode_inv) cairo_set_source_rgb (cr, 0.8, 0.8, 0.8);
	else cairo_set_source_rgb (cr, 1-0.8, 1-0.8, 1-0.8);
	cairo_stroke (cr);
	
	int altitu = priv->altitude *1000;

	cairo_move_to (cr,x-0.427*radius,y-0.29*radius);	// X00000
	sprintf(str,"%d",altitu / 100000000 % 10);
	cairo_show_text (cr, str);
	cairo_move_to (cr,x-0.282*radius,y-0.29*radius);	// 0X0000
	sprintf(str,"%d",altitu / 10000000 % 10);
	cairo_show_text (cr, str);
	cairo_move_to (cr,x-0.137*radius,y-0.29*radius);	// 00X000
	sprintf(str,"%d",altitu / 1000000 % 10);
	cairo_show_text (cr, str);
	cairo_move_to (cr,x+0.008*radius,y-0.29*radius);	// 000X00
	sprintf(str,"%d",altitu / 100000 % 10);
	cairo_show_text (cr, str);
	cairo_move_to (cr,x+0.153*radius,y-0.29*radius);	// 0000X0
	sprintf(str,"%d",altitu / 10000 % 10);
	cairo_show_text (cr, str);
	cairo_move_to (cr,x+0.298*radius,y-0.29*radius);	// 00000X
	sprintf(str,"%d",altitu / 1000 % 10);
	cairo_show_text (cr, str);
	cairo_stroke (cr);
	
	// 10 thousand hand
	cairo_save (cr);
	if(!priv->color_mode_inv) cairo_set_source_rgb (cr, 1, 1, 1);
	else cairo_set_source_rgb (cr, 0, 0, 0);
	cairo_set_line_width (cr, 2);
	cairo_move_to (cr, x, y);
	cairo_line_to (cr, x + radius * sin (5 * M_PI / 25
							 * (altitu / (10000000/factor) % 10) + M_PI / 50 
							 * (altitu / (1000000/factor) % 10) + M_PI / 500 
							 * (altitu / (100000/factor) % 10) + M_PI / 5000 
							 * (altitu / (10000/factor) % 10) + M_PI / 50000 
							 * (altitu / (1000/factor) % 10)),
							 y + radius * -cos (5 * M_PI / 25
							 * (altitu / (10000000/factor) % 10) + M_PI / 50 
							 * (altitu / (1000000/factor) % 10) + M_PI / 500 
							 * (altitu / (100000/factor) % 10) + M_PI / 5000 
							 * (altitu / (10000/factor) % 10) + M_PI / 50000 
							 * (altitu / (1000/factor) % 10)));
							 
	cairo_move_to (cr, x + (radius-0.1*radius) * sin (5 * M_PI / 25
							 * (altitu / (10000000/factor) % 10) + M_PI / 50 
							 * (altitu / (1000000/factor) % 10) + M_PI / 500 
							 * (altitu / (100000/factor) % 10) + M_PI / 5000 
							 * (altitu / (10000/factor) % 10) + M_PI / 50000 
							 * (altitu / (1000/factor) % 10)),
							 y + (radius-0.1*radius) * -cos (5 * M_PI / 25
							 * (altitu / (10000000/factor) % 10) + M_PI / 50 
							 * (altitu / (1000000/factor) % 10) + M_PI / 500 
							 * (altitu / (100000/factor) % 10) + M_PI / 5000 
							 * (altitu / (10000/factor) % 10) + M_PI / 50000 
							 * (altitu / (1000/factor) % 10)));
	cairo_line_to (cr, x + radius * sin (M_PI / 50 + 5 * M_PI / 25
							 * (altitu / (10000000/factor) % 10) + M_PI / 50 
							 * (altitu / (1000000/factor) % 10) + M_PI / 500 
							 * (altitu / (100000/factor) % 10) + M_PI / 5000 
							 * (altitu / (10000/factor) % 10) + M_PI / 50000 
							 * (altitu / (1000/factor) % 10)),
							 y + radius * -cos (M_PI / 50 + 5 * M_PI / 25
							 * (altitu / (10000000/factor) % 10) + M_PI / 50 
							 * (altitu / (1000000/factor) % 10) + M_PI / 500 
							 * (altitu / (100000/factor) % 10) + M_PI / 5000 
							 * (altitu / (10000/factor) % 10) + M_PI / 50000 
							 * (altitu / (1000/factor) % 10)));
	cairo_line_to (cr, x + radius * sin (-M_PI / 50 + 5 * M_PI / 25
							 * (altitu / (10000000/factor) % 10) + M_PI / 50 
							 * (altitu / (1000000/factor) % 10) + M_PI / 500 
							 * (altitu / (100000/factor) % 10) + M_PI / 5000 
							 * (altitu / (10000/factor) % 10) + M_PI / 50000 
							 * (altitu / (1000/factor) % 10)),
							 y + radius * -cos (-M_PI/50 + 5 * M_PI / 25
							 * (altitu / (10000000/factor) % 10) + M_PI / 50 
							 * (altitu / (1000000/factor) % 10) + M_PI / 500 
							 * (altitu / (100000/factor) % 10) + M_PI / 5000 
							 * (altitu / (10000/factor) % 10) + M_PI / 50000 
							 * (altitu / (1000/factor)% 10)));							 
	cairo_move_to (cr, x + (radius-0.1*radius) * sin (5 * M_PI / 25
							 * (altitu / (10000000/factor) % 10) + M_PI / 50 
							 * (altitu / (1000000/factor) % 10) + M_PI / 500 
							 * (altitu / (100000/factor) % 10) + M_PI / 5000 
							 * (altitu / (10000/factor) % 10) + M_PI / 50000 
							 * (altitu / (1000/factor) % 10)),
							 y + (radius-0.1*radius) * -cos (5 * M_PI / 25
							 * (altitu / (10000000/factor) % 10) + M_PI / 50 
							 * (altitu / (1000000/factor) % 10) + M_PI / 500 
							 * (altitu / (100000/factor) % 10) + M_PI / 5000 
							 * (altitu / (10000/factor) % 10) + M_PI / 50000 
							 * (altitu / (1000/factor) % 10)));							 				
	cairo_fill_preserve (cr);
	cairo_stroke (cr);
	cairo_restore (cr);
		

	// thousand hand
	cairo_save (cr);
	if(!priv->color_mode_inv) cairo_set_source_rgb (cr, 1, 1, 1);
	else cairo_set_source_rgb (cr, 0, 0, 0); 
	cairo_set_line_width (cr, 0.03*radius);
	cairo_move_to (cr, x, y);
	
	cairo_line_to (cr, x + (radius - 0.7 * radius) * sin (5 * M_PI / 25
							 * (altitu / (1000000/factor) % 10) + M_PI / 50 
							 * (altitu / (100000/factor) % 10) + M_PI / 500 
							 * (altitu / (10000/factor) % 10) + M_PI / 5000 
							 * (altitu / (1000/factor) % 10) + M_PI / 50000 
							 * (altitu / (100/factor) % 10)),
							 y + (radius - 0.7 * radius) * -cos (5 * M_PI / 25
							 * (altitu / (1000000/factor) % 10) + M_PI / 50 
							 * (altitu / (100000/factor) % 10) + M_PI / 500 
							 * (altitu / (10000/factor) % 10) + M_PI / 5000 
							 * (altitu / (1000/factor) % 10) + M_PI / 50000 
							 * (altitu / (100/factor) % 10)));
	cairo_stroke (cr);
	cairo_restore (cr);
	
	cairo_save (cr);
	if(!priv->color_mode_inv) cairo_set_source_rgb (cr, 1, 1, 1);
	else cairo_set_source_rgb (cr, 0, 0, 0); 
	cairo_set_line_width (cr, 1);
	cairo_move_to (cr, x, y);
	cairo_line_to (cr, x + radius / 3 * sin (M_PI/15 + 5 * M_PI / 25
							 * (altitu / (1000000/factor) % 10) + M_PI / 50 
							 * (altitu / (100000/factor) % 10) + M_PI / 500 
							 * (altitu / (10000/factor) % 10) + M_PI / 5000 
							 * (altitu / (1000/factor) % 10) + M_PI / 50000 
							 * (altitu / (100/factor) % 10)),
							 y + radius / 3 * -cos (M_PI/15 + 5 * M_PI / 25
							 * (altitu / (1000000/factor) % 10) + M_PI / 50 
							 * (altitu / (100000/factor) % 10) + M_PI / 500 
							 * (altitu / (10000/factor) % 10) + M_PI / 5000 
							 * (altitu / (1000/factor) % 10) + M_PI / 50000 
							 * (altitu / (100/factor) % 10)));
	cairo_line_to (cr, x + radius / 2 * sin (5 * M_PI / 25
							 * (altitu / (1000000/factor) % 10) + M_PI / 50 
							 * (altitu / (100000/factor) % 10) + M_PI / 500 
							 * (altitu / (10000/factor) % 10) + M_PI / 5000 
							 * (altitu / (1000/factor) % 10) + M_PI / 50000 
							 * (altitu / (100/factor) % 10)),
							 y + radius / 2 * -cos (5 * M_PI / 25
							 * (altitu / (1000000/factor) % 10) + M_PI / 50 
							 * (altitu / (100000/factor) % 10) + M_PI / 500 
							 * (altitu / (10000/factor) % 10) + M_PI / 5000 
							 * (altitu / (1000/factor) % 10) + M_PI / 50000 
							 * (altitu / (100/factor) % 10)));
	cairo_line_to (cr, x + radius / 3 * sin (-M_PI/15 + 5 * M_PI / 25
							 * (altitu / (1000000/factor) % 10) + M_PI / 50 
							 * (altitu / (100000/factor) % 10) + M_PI / 500 
							 * (altitu / (10000/factor) % 10) + M_PI / 5000 
							 * (altitu / (1000/factor) % 10) + M_PI / 50000 
							 * (altitu / (100/factor) % 10)),
							 y + radius / 3 * -cos (-M_PI/15 + 5 * M_PI / 25
							 * (altitu / (1000000/factor) % 10) + M_PI / 50 
							 * (altitu / (100000/factor) % 10) + M_PI / 500 
							 * (altitu / (10000/factor) % 10) + M_PI / 5000 
							 * (altitu / (1000/factor) % 10) + M_PI / 50000 
							 * (altitu / (100/factor) % 10)));
	cairo_line_to (cr, x, y);
	cairo_fill_preserve (cr);
	cairo_stroke (cr);
	cairo_restore (cr);
	
	cairo_save (cr);
	if(!priv->color_mode_inv) cairo_set_source_rgb (cr, 0, 0, 0);
	else cairo_set_source_rgb (cr, 1, 1, 1);
	cairo_arc (cr, x,	y, radius-0.86*radius, M_PI/3 + 5 * M_PI / 25
							 * (altitu / (1000000/factor) % 10) + M_PI / 50 
							 * (altitu / (100000/factor) % 10) + M_PI / 500 
							 * (altitu / (10000/factor) % 10) + M_PI / 5000 
							 * (altitu / (1000/factor) % 10) + M_PI / 50000 
							 * (altitu / (100/factor) % 10), 
							 2 * M_PI / 3 + 5 * M_PI / 25
							 * (altitu / (1000000/factor) % 10) + M_PI / 50 
							 * (altitu / (100000/factor) % 10) + M_PI / 500 
							 * (altitu / (10000/factor) % 10) + M_PI / 5000 
							 * (altitu / (1000/factor) % 10) + M_PI / 50000 
							 * (altitu / (100/factor) % 10));
	cairo_line_to (cr, x, y);
	cairo_fill_preserve (cr);
	cairo_stroke (cr);
	cairo_restore (cr);
			
	// hundred hand
	if(factor==100)
	{
		cairo_save (cr);
		if(!priv->color_mode_inv) cairo_set_source_rgb (cr, 1, 1, 1);
		else cairo_set_source_rgb (cr, 0, 0, 0); 
		cairo_set_line_width (cr, 0.03*radius);
		cairo_move_to (cr, x, y);							
		cairo_line_to (cr, x + (radius - 0.2 * radius) * sin (5 * M_PI / 25
								 * (altitu / (100000/factor) % 10) + M_PI / 50 
								 * (altitu / (10000/factor) % 10) + M_PI / 500 
								 * (altitu / (1000/factor) % 10) + M_PI / 5000 
								 * (altitu / (100/factor) % 10)),
								 y + (radius - 0.2 * radius) * -cos (5 * M_PI / 25
								 * (altitu / (100000/factor) % 10) + M_PI / 50 
								 * (altitu / (10000/factor) % 10) + M_PI / 500 
								 * (altitu / (1000/factor) % 10) + M_PI / 5000 
								 * (altitu / (100/factor) % 10)));
		cairo_stroke (cr);
		cairo_restore (cr);
		
		cairo_save (cr);
		if(!priv->color_mode_inv) cairo_set_source_rgb (cr, 0, 0, 0);
		else cairo_set_source_rgb (cr, 1, 1, 1);
		cairo_set_line_width (cr, 0.03*radius);
		cairo_move_to (cr, x - (radius - 0.8 * radius) * sin (5 * M_PI / 25
 								 * (altitu / (100000/factor) % 10) + M_PI / 50 
								 * (altitu / (10000/factor) % 10) + M_PI / 500 
								 * (altitu / (1000/factor) % 10) + M_PI / 5000 
								 * (altitu / (100/factor) % 10)),
								 y - (radius - 0.8 * radius) * -cos (5 * M_PI / 25
								 * (altitu / (100000/factor) % 10) + M_PI / 50 
								 * (altitu / (10000/factor) % 10) + M_PI / 500 
								 * (altitu / (1000/factor) % 10) + M_PI / 5000 
								 * (altitu / (100/factor) % 10)));
		cairo_arc (cr, x - (radius - 0.8 * radius) * sin (5 * M_PI / 25
							* (altitu / (100000/factor) % 10) + M_PI / 50 
							* (altitu / (10000/factor) % 10) + M_PI / 500 
							* (altitu / (1000/factor) % 10) + M_PI / 5000 
							* (altitu / (100/factor) % 10)),
							y - (radius - 0.8 * radius) * -cos(5 * M_PI / 25
							* (altitu / (100000/factor) % 10) + M_PI / 50 
							* (altitu / (10000/factor) % 10) + M_PI / 500 
							* (altitu / (1000/factor) % 10) + M_PI / 5000 
							* (altitu / (100/factor) % 10)),
							radius-0.98*radius, 0, 2 * M_PI);
		cairo_move_to (cr, x - (radius - 0.8 * radius) * sin (5 * M_PI / 25
								 * (altitu / (100000/factor) % 10) + M_PI / 50 
								 * (altitu / (10000/factor) % 10) + M_PI / 500 
								 * (altitu / (1000/factor) % 10) + M_PI / 5000 
								 * (altitu / (100/factor) % 10)),
								 y - (radius - 0.8 * radius) * -cos (5 * M_PI / 25
								 * (altitu / (100000/factor) % 10) + M_PI / 50 
								 * (altitu / (10000/factor) % 10) + M_PI / 500 
								 * (altitu / (1000/factor) % 10) + M_PI / 5000 
								 * (altitu / (100/factor) % 10)));						
		cairo_line_to (cr, x, y);
		cairo_stroke (cr);
		cairo_restore (cr);
	}
	else
	{
		cairo_save (cr);
		if(!priv->color_mode_inv) cairo_set_source_rgb (cr, 1, 1, 1);
		else cairo_set_source_rgb (cr, 0, 0, 0); 
		cairo_set_line_width (cr, 0.03*radius);
		cairo_move_to (cr, x, y);							
		cairo_line_to (cr, x + (radius - 0.2 * radius) * sin (5 * M_PI / 25
								 * (altitu / (100000/factor) % 10) + M_PI / 50 
								 * (altitu / (10000/factor) % 10) + M_PI / 500 
								 * (altitu / (1000/factor) % 10) + M_PI / 5000 
								 * (altitu / (100/factor) % 10) + M_PI / 50000 
								 * (altitu / (10/factor) % 10)),
								 y + (radius - 0.2 * radius) * -cos (5 * M_PI / 25
								 * (altitu / (100000/factor) % 10) + M_PI / 50 
								 * (altitu / (10000/factor) % 10) + M_PI / 500 
								 * (altitu / (1000/factor) % 10) + M_PI / 5000 
								 * (altitu / (100/factor) % 10) + M_PI / 50000 
								 * (altitu / (10/factor) % 10)));
		cairo_stroke (cr);
		cairo_restore (cr);
		
		cairo_save (cr);
		if(!priv->color_mode_inv) cairo_set_source_rgb (cr, 0, 0, 0);
		else cairo_set_source_rgb (cr, 1, 1, 1);
		cairo_set_line_width (cr, 0.03*radius);
		cairo_move_to (cr, x - (radius - 0.8 * radius) * sin (5 * M_PI / 25
 								 * (altitu / (100000/factor) % 10) + M_PI / 50 
								 * (altitu / (10000/factor) % 10) + M_PI / 500 
								 * (altitu / (1000/factor) % 10) + M_PI / 5000 
								 * (altitu / (100/factor) % 10) + M_PI / 50000 
							    * (altitu / (10/factor) % 10)),
								 y - (radius - 0.8 * radius) * -cos (5 * M_PI / 25
								 * (altitu / (100000/factor) % 10) + M_PI / 50 
								 * (altitu / (10000/factor) % 10) + M_PI / 500 
								 * (altitu / (1000/factor) % 10) + M_PI / 5000 
								 * (altitu / (100/factor) % 10) + M_PI / 50000 
								 * (altitu / (10/factor) % 10)));
		cairo_arc (cr, x - (radius - 0.8 * radius) * sin (5 * M_PI / 25
							* (altitu / (100000/factor) % 10) + M_PI / 50 
							* (altitu / (10000/factor) % 10) + M_PI / 500 
							* (altitu / (1000/factor) % 10) + M_PI / 5000 
							* (altitu / (100/factor) % 10) + M_PI / 50000 
							* (altitu / (10/factor) % 10)),
							y - (radius - 0.8 * radius) * -cos(5 * M_PI / 25
							* (altitu / (100000/factor) % 10) + M_PI / 50 
							* (altitu / (10000/factor) % 10) + M_PI / 500 
							* (altitu / (1000/factor) % 10) + M_PI / 5000 
							* (altitu / (100/factor) % 10) + M_PI / 50000 
							* (altitu / (10/factor) % 10)),
							radius-0.98*radius, 0, 2 * M_PI);
		cairo_move_to (cr, x - (radius - 0.8 * radius) * sin (5 * M_PI / 25
								 * (altitu / (100000/factor) % 10) + M_PI / 50 
								 * (altitu / (10000/factor) % 10) + M_PI / 500 
								 * (altitu / (1000/factor) % 10) + M_PI / 5000 
								 * (altitu / (100/factor) % 10) + M_PI / 50000 
								 * (altitu / (10/factor) % 10)),
								 y - (radius - 0.8 * radius) * -cos (5 * M_PI / 25
								 * (altitu / (100000/factor) % 10) + M_PI / 50 
								 * (altitu / (10000/factor) % 10) + M_PI / 500 
								 * (altitu / (1000/factor) % 10) + M_PI / 5000 
								 * (altitu / (100/factor) % 10) + M_PI / 50000 
								 * (altitu / (10/factor) % 10)));						
		cairo_line_to (cr, x, y);
		cairo_stroke (cr);
		cairo_restore (cr);
	}
		
	// centre cercle 
	if(!priv->color_mode_inv) cairo_set_source_rgb (cr, 1, 1, 1);
	else cairo_set_source_rgb (cr, 0, 0, 0);
	cairo_arc (cr, x, y, radius-0.98*radius, 0, 2 * M_PI);
	cairo_fill_preserve (cr);
	cairo_stroke (cr);
}

static gboolean
gtk_altimeter_expose (GtkWidget *alt, GdkEventExpose *event)
{
	cairo_t *cr;

	/* get a cairo_t */
	cr = gdk_cairo_create (alt->window);

	cairo_rectangle (cr,
			event->area.x, event->area.y,
			event->area.width, event->area.height);
	cairo_clip (cr);
	
	draw (alt, cr);

	cairo_destroy (cr);

	return FALSE;
}

static void
gtk_altimeter_redraw_canvas (GtkAltimeter *alt)
{
	GtkWidget *widget;
	GdkRegion *region;
	
	widget = GTK_WIDGET (alt);

	if (!widget->window) return;

	region = gdk_drawable_get_clip_region (widget->window);
	/* redraw the cairo canvas completely by exposing it */
	gdk_window_invalidate_region (widget->window, region, TRUE);
	gdk_window_process_updates (widget->window, TRUE);

	gdk_region_destroy (region);
}

static gboolean
gtk_altimeter_update (gpointer data)
{
	GtkAltimeter *alt;
	GtkAltimeterPrivate *priv;

	alt = GTK_ALTIMETER (data);
	priv = GTK_ALTIMETER_GET_PRIVATE (alt);
		
	gtk_altimeter_redraw_canvas (alt);

	return TRUE; /* keep running this event */
}

GtkWidget *
gtk_altimeter_new (void)
{
	return g_object_new (GTK_ALTIMETER_TYPE, NULL);
}

// widget finction
void
gtk_altimeter_set_unit(GtkWidget *widget, gboolean unit_type)
{
	GtkAltimeterPrivate *priv;
	priv = GTK_ALTIMETER_GET_PRIVATE (GTK_ALTIMETER(widget));
   priv->unit_is_feet=unit_type;
   //gtk_altimeter_update(widget);
}

void
gtk_altimeter_set_unit_value(GtkWidget *widget, gint val)
{
	GtkAltimeterPrivate *priv;
	priv = GTK_ALTIMETER_GET_PRIVATE (GTK_ALTIMETER(widget));
   priv->unit_value = val;
   //gtk_altimeter_update(widget);
}

void gtk_altimeter_set_color_mode(GtkWidget *widget, gboolean col)
{
	GtkAltimeterPrivate *priv;
	priv = GTK_ALTIMETER_GET_PRIVATE (GTK_ALTIMETER(widget));
	priv->color_mode_inv = col;
   //gtk_altimeter_update(widget);
}

void
gtk_altimeter_set_alti(GtkWidget *widget, gdouble altitude)
{
	GtkAltimeterPrivate *priv;
	priv = GTK_ALTIMETER_GET_PRIVATE (GTK_ALTIMETER(widget));
   priv->altitude = altitude;
   //gtk_altimeter_update(widget);
}
