#ifndef OBJECT_DETECTION_OBJECTS_HPP__
#define OBJECT_DETECTION_OBJECTS_HPP__

#include "Pose.hpp"
#include <boost/shared_ptr.hpp>
#include <stdexcept>
#include <inttypes.h>
#include <vector>

namespace object_detection
{

/*
 * @brief specifies the color of an object or surface
 *
 * For now this is very simple, and internally holds an rgb representation of
 * the color.  In reality this is of course much more complicated since
 * different color profiles of the sensor/display devices have to be taken into
 * account. So the representation can be considered canonical.
 *
 * @todo once this class is matured, move it into base/types.
 */ 
struct Color
{
    /** red channel */
    float r;
    /** green channel */
    float g;
    /** blue channel */
    float b;

    /** default constructor returns black */
    Color() : r(.0f), g(.0f), b(.0f) {}

    /** construct based on rgb values */
    Color( float r, float g, float b )
	: r( r ), g( g ), b( b ) {}

    /** create Color from from 8bit RGB
     */
    static Color fromRGB8( uint8_t r, uint8_t g, uint8_t b )
    {
	const float max = 255.0f;
	return Color( r/max, g/max, b/max );
    }

    static Color fromHSV( float h, float s, float v )
    {
	throw std::runtime_error("not implemented");
    }

    /** to RGB color space
     *
     * no conversion is needed since this is the canonical representation.
     */
    void toRGB( float &r, float &g, float &b )
    {
	r = this->r;
	g = this->g;
	b = this->b;
    }

    void toHSV( float &h, float &s, float &v )
    {
	throw std::runtime_error("not implemented");
    }

    /** to RGB color space
     *
     * no conversion is needed since this is the canonical representation.
     */
    void toRGB8( uint8_t &r, uint8_t &g, uint8_t &b )
    {
        const uint8_t max = 255;
        r = this->r * max;
        g = this->g * max;
        b = this->b * max;
    }

    /**
     *  convert RGB to normalized RGB color space
     *  TODO:range
     */
    void toNRGB( float &nr, float &ng, float &nb)
    {
        float sum = r + g + b;
        if ( sum != 0.0 )
        {
            nr = r / sum;
            ng = g / sum;
            nb = b / sum;
        }
        else
            nr = ng = nb = 0.0;
    }

    // use opencv conversion
    // http://docs.opencv.org/modules/imgproc/doc/miscellaneous_transformations.html#cvtcolor
    void toXYZ(float &x, float &y, float &z)
    {
        float c[3] = {this->r, this->g, this->b};

        x = 0.412453 * c[0] + 0.357580 * c[1] + 0.180423 * c[2];
        y = 0.212671 * c[0] + 0.715160 * c[1] + 0.072169 * c[2];
        z = 0.019334 * c[0] + 0.119193 * c[1] + 0.950227 * c[2];
    }

    // use opencv conversion
    // http://docs.opencv.org/modules/imgproc/doc/miscellaneous_transformations.html#cvtcolor
    static Color fromXYZ(float x, float y, float z)
    {
        float r = 3.240479 * x + -1.53715 * y + -0.498538 * z;
        float g = -0.969256 * x + 1.875991 * y + 0.041556 * z;
        float b = 0.055648 * x + -0.204043 * y + 1.057311 * z;
        return Color(r, g, b);
    }

    // use opencv conversion
    // http://docs.opencv.org/modules/imgproc/doc/miscellaneous_transformations.html#cvtcolor 
    // range
    // 0 <= L <= 100
    // -127 <= a <= 127
    // -127 <= b <= 127      
    void toLab(float &l, float &a, float &b)
    {
        float c[3] = {this->r, this->g, this->b};

        for (unsigned int i = 0; i < 3; ++i)
        {
            if (c[i] > 0.04045)
                c[i] = std::pow(((c[i] + 0.055) / 1.055), 2.4);
            else
                c[i] = c[i] / 12.92;
        }

        float x = 0.412453 * c[0] + 0.357580 * c[1] + 0.180423 * c[2];
        float y = 0.212671 * c[0] + 0.715160 * c[1] + 0.072169 * c[2];
        float z = 0.019334 * c[0] + 0.119193 * c[1] + 0.950227 * c[2];

        c[0] = x;
        c[1] = y;
        c[2] = z;

        float cn[3] = {0.950456, 1.0, 1.088754};
        
        for (unsigned int i = 0; i < 3; ++i)
        {
            c[i] = c[i] / cn[i];
            if (c[i] > 0.008856)
                c[i] = std::pow(c[i], 1./3.);
            else 
                c[i] = (7.787 * c[i]) + (16. / 116.);            
        }

        l = (116. * c[1]) - 16.;    
        a = 500. * (c[0] - c[1]);
        b = 200. * (c[1] - c[2]);
    }

    // range [0;255] for l, a and b
    void toLab8(uint8_t &l, uint8_t &a, uint8_t &b)
    {
        float l_float = 0.;
        float a_float = 0.;
        float b_float = 0.;

        toLab(l_float, a_float, b_float);

        l = l_float * 255 / 100;
        a = a_float + 128;
        b = b_float + 128;
    }

    // use opencv conversion
    // http://docs.opencv.org/modules/imgproc/doc/miscellaneous_transformations.html#cvtcolor 
    // 0 <= L <= 100
    // -127 <= a <= 127
    // -127 <= b <= 127  
    static Color fromLab(float l, float a, float b)
    {
        float c[3];
        c[1] = (l + 16.) / 116.;        // y
        c[0] = a / 500. + c[1];      // x 
        c[2] = c[1] - b / 200.;      // z

        float cn[3] = {0.950456, 1.0, 1.088754};    

        for (unsigned int i = 0; i < 3; ++i)
        {
             if ( std::pow(c[i], 3.) > 0.008856)
                c[i] = std::pow(c[i], 3.);
            else
                c[i] = (c[i] - 16. / 116.) / 7.787;

            c[i] = c[i] * cn[i];
        }

        float red = 3.240479 * c[0] + -1.53715 * c[1] + -0.498538 * c[2];
        float green = -0.969256 * c[0] + 1.875991 * c[1] + 0.041556 * c[2];
        float blue = 0.055648 * c[0] + -0.204043 * c[1] + 1.057311 * c[2];        

        c[0] = red;
        c[1] = green;
        c[2] = blue;

        for (unsigned int i = 0; i < 3; ++i)
        {
            if (c[i] > 0.0031308)
                c[i] = 1.055 * std::pow(c[i], 1./2.4) - 0.055;
        }

        return Color(c[0], c[1], c[2]);
    }

    // l, a, b range [0; 255]
    static Color fromLab8(uint8_t l, uint8_t a, uint8_t b)
    {
        float l_float = (float)l * 100. / 255.;
        float a_float = (float)a - 128.;
        float b_float = (float)b - 128.;
        return Color::fromLab(l_float, a_float, b_float);
    }
};

/** 
 * @brief base class for an object which can be detected by the library
 */
struct Object
{
    typedef boost::shared_ptr<Object> Ptr;

    /** position of the object*/
    base::Vector3d position;
    base::Quaterniond orientation;
    base::Vector3d axis;//axis vector

    base::Affine3d getTransform() const
    {
	return base::Pose( position, orientation ).toTransform();
    }

    /** overall likelihoof of object match **/
    double likelihood;

    /** how good does the shape match the shape model **/
    double shapeMatch;

    /** color of the object */
    Color color;

    /** get a point sampling of the object */
    virtual std::vector<base::Vector3d> getSampling( double density ) = 0;

    /** @return the diameter of the sphere which encloses the object */
    virtual float boundingSphereDiameter() const = 0;

    /** @return diameter of the inner bounding sphere. This is a lower limit
     * for the objects cross-sectional shapes */
    virtual float innerSphereDiameter() const = 0;
};

struct Box : public Object
{
    typedef boost::shared_ptr<Box> Ptr;

    /** size of the box
     * all dimensions are given in m
     */
    base::Vector3d dimensions;

    virtual float boundingSphereDiameter() const
    {
	return dimensions.norm();
    }

    virtual float innerSphereDiameter() const
    {
	return dimensions.minCoeff();
    }

    virtual std::vector<base::Vector3d> getSampling( double density )
    {
	std::vector<base::Vector3d> res;
	for( int dim = 0; dim < 3; dim++ )
	{
	    int dimx = (dim==0) ? 1 : 0;
	    int dimy = (dim==0||dim==1) ? 2 : 1;
	    for( double px = -dimensions[dimx]/2.0; px < dimensions[dimx]/2.0; px += density )
	    {
		for( double py = -dimensions[dimy]/2.0; py < dimensions[dimy]/2.0; py += density )
		{
		    base::Vector3d v;
		    v[dimx] = px;
		    v[dimy] = py;
		    v[dim] = dimensions[dim]/2.0;
		    res.push_back( v );
		    v[dim] = -dimensions[dim]/2.0;
		    res.push_back( v );
		}
	    }
	}

	return res;
    }
};

struct Cylinder : public Object
{
    typedef boost::shared_ptr<Cylinder> Ptr;

    /** diameter of the cylinder in m */
    float diameter;
    /** height of the cylinder in m */
    float height;

    virtual float boundingSphereDiameter() const
    {
	return Eigen::Vector2d( diameter, height ).norm();
    }

    virtual float innerSphereDiameter() const
    {
	return std::min( diameter, height );
    }

    virtual std::vector<base::Vector3d> getSampling( double density )
    {
	std::vector<base::Vector3d> res;

	for( double c = 0; c < 2.0*M_PI; c += density / diameter / 2.0 )
	{
            double x = cos( c );
            double y = sin( c );
            for( double z = -height/2.0; z < height/2.0; z += density )
            {
                const float r = diameter / 2.0;
                res.push_back( base::Vector3d( x * r, y * r, z ) );
            }
            for( double r = 0; r < diameter / 2.0; r += density )
            {
                res.push_back( base::Vector3d( x * r, y * r, height/2.0 ) );
                res.push_back( base::Vector3d( x * r, y * r, -height/2.0 ) );
            }
	}

	return res;
    }
};


/** primitive type */
    enum types 
    {
	INVALID = 0,
	BOX,
	CYLINDER
    };

/** 
 * Transport class for primitive objects, which
 * is a flat representation that is compatible with typelib
 * and can be copied.
 */
struct PrimitiveObject
{

    /** type of the the primitive */
    types type;

    /** parameters for the primitive types */
    base::Vector4d param;

    base::Vector3d position;

    base::Quaterniond orientation;

    double likelihood;

    double shapeMatch;

    base::Vector3d color;
    

    PrimitiveObject(){
       param = base::Vector4d::Zero(); 
    }

    /** 
     * construct a PrimitiveObject from an Object.
     * 
     * Effectively this is a marshaling of the primitive type hirarchie to a
     * flat structure.
     */
    explicit PrimitiveObject( const Object &o )
    {
        {
            const Box *box = dynamic_cast<const Box*>(&o);
            if( box )
            {
            type = BOX;
            param.head<3>() = box->dimensions;
            }
        }
        {
            const Cylinder *cyl = dynamic_cast<const Cylinder*>(&o);
            if( cyl )
            {
            type = CYLINDER;
            param.x() = cyl->diameter;
            param.y() = cyl->height;
            }
        }

        position = o.position;
        orientation = o.orientation;
        likelihood = o.likelihood;
        shapeMatch = o.shapeMatch;

        color.x() = o.color.r;
        color.y() = o.color.g;
        color.z() = o.color.b;
    }

    Box getBox() const
    {
	assert( type == BOX );
	Box box;
	box.dimensions = param.head<3>();

        box.position = position;
        box.orientation = orientation;
        box.likelihood = likelihood;
        box.shapeMatch = shapeMatch;

        box.color.r = color.x();
        box.color.g = color.y();
        box.color.b = color.z();

	return box;
    }

    Cylinder getCylinder() const
    {
	assert( type == CYLINDER );
	Cylinder cyl;	
	cyl.diameter = param.x();
	cyl.height = param.y();

        cyl.position = position;
        cyl.orientation = orientation;
        cyl.likelihood = likelihood;
        cyl.shapeMatch = shapeMatch;

        cyl.color.r = color.x();
        cyl.color.g = color.y();
        cyl.color.b = color.z();

	return cyl;
    }

    /** 
     * return an object pointer
     */
    Object::Ptr getObject() const
    {
        Object::Ptr res;
        switch( type )
        {
            case BOX:
            {
            Box *box = new Box();
            box->dimensions = param.head<3>();
            res = Object::Ptr(box);
            break;
            }
            case CYLINDER:
            {
            Cylinder *cyl = new Cylinder();
            cyl->diameter = param.x();
            cyl->height = param.y();
            res = Object::Ptr(cyl);
            break;
            }
            default:
            throw std::runtime_error( "Object type invalid" );
        }

        res->position = position;
        res->orientation = orientation;
        res->likelihood = likelihood;
        res->shapeMatch = shapeMatch;

        res->color.r = color.x();
        res->color.g = color.y();
        res->color.b = color.z();

        return res;
    }
};

};
#endif
