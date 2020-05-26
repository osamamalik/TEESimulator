#include "vtkAutoInit.h"
#define vtkRenderingCore_AUTOINIT 2(vtkRenderingOpenGL2, vtkInteractionStyle)

#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProgrammableFilter.h>
#include <vtkCallbackCommand.h>
#include <vtkNamedColors.h>
#include <vtkProperty.h>

#include <vtkImageData.h>
#include <vtkMetaImageReader.h>
#include <vtkInteractorStyleImage.h>
#include <vtkImageActor.h>
#include <vtkImageMapper3D.h>
#include <vtkImageReslice.h>
#include <vtkImageViewer2.h>

#include "serial.h"
#include <iostream>
#include <string>
#include <regex>
#include <unistd.h>
#include <sstream>

#include "vtkCT2USSimulation.h"

void TimerCallbackFunction ( vtkObject* caller, long unsigned int eventId, void* clientData, void* callData );

/*---- Globals ----*/

std::string serialPort = "/dev/cu.usbserial-A506B21I";       // Port name is dependant on the computer
Serial serial(serialPort);

// x- and y-coordinates
double x = 0;
double y = 0;
double angle = 0;

std::string scans[30];

vtkSmartPointer<vtkMetaImageReader> reader;
vtkSmartPointer<vtkImageActor> imageActor;

/*-----------------*/

/*  Determines if input string is a number  */
bool isNumber(std::string x)
{
    std::regex e ("^-?\\d+");
    return std::regex_match (x,e);
}

/*  Reads Serial data and updates x- and y-coordinate variables   */
void GetSerialData()
{
    char c = ' ';
    std::string coords = "";
    int i = 0;
    while (serial.Available() > 0 && c != '!')
    {
        serial.Read(&c);
        if (c != '!') {     // '!' character indicates end of current x and y coordinates
            coords += c;
        }
        else {
            break;
        }
        i++;
    }
    
    std::size_t found = coords.find_first_of("0123456789-");

    // Only change x and y values when new, valid values are read from serial data
    if (found < 10) {
        
        // parse coordinate string into separate x and y coordinates
        bool endOfXString = false;
        bool endOfYString = false;
        int ci = 0;     // character index
        std::string xpos = "";
        std::string ypos = "";
        while (!endOfXString) {
            char c = coords[ci];
            if (c == '-' || c == '0' || c == '1' || c == '2' || c == '3' || c == '4' || c == '5' || c == '6' || c == '7' || c == '8' || c == '9') {
                xpos += coords[ci];
            }
            ci++;
            if (c == ' ') {
                endOfXString = true;
            }
            
        }
        while (!endOfYString) {
            char c = coords[ci];
            if (c == '-' || c == '0' || c == '1' || c == '2' || c == '3' || c == '4' || c == '5' || c == '6' || c == '7' || c == '8' || c == '9') {
                ypos += coords[ci];
                ci++;
            }
            else {
                endOfYString = true;
            }
        }
        
        if (isNumber(xpos)) {
            x = std::stod(xpos) * -1;
        }
        if (isNumber(ypos)) {
            y = std::stod(ypos) * -1;
        }
    }
    // Print x and y coordinates to console
    std::cout << "x: " << x << std::endl << "y: " << y << std::endl << "------------" << std::endl;
}

/*  Returns the current scan by checking the z-value of each image.
    z-values have been obtained from each individual .mhd file.
    This method can be improved if there is a method that obtains
    the value for the current image.    */
std::string GetNewScan() {
    std::string scan = "";
    
    if (y >= 0 && y < 4.2) {
        scan = scans[0];
    }
    else if (y >= 4.3 & y < 8.4) {
        scan = scans[1];
    }
    else if (y >= 8.5 && y < 12.6) {
        scan = scans[2];
    }
    else if (y >= 12.7 && y < 16.2) {
        scan = scans[3];
    }
    else if (y >= 16.3 && y < 20.4) {
        scan = scans[4];
    }
    else if (y >= 20.5 && y < 29.4) {
        scan = scans[5];
    }
    else if (y >= 29.5 && y < 33.7) {
        scan = scans[6];
    }
    else if (y >= 33.8 && y < 37.9) {
        scan = scans[7];
    }
    else if (y >= 38.0 && y < 42.0) {
        scan = scans[8];
    }
    else if (y >= 42.1 && y < 45.7) {
        scan = scans[9];
    }
    else if (y >= 458 && y < 49.9) {
        scan = scans[10];
    }
    else if (y >= 50.0 && y < 54.1) {
        scan = scans[11];
    }
    else if (y >= 54.2 && y < 61.6) {
        scan = scans[12];
    }
    else if (y >= 61.7 && y < 65.8) {
        scan = scans[13];
    }
    else if (y >= 65.9 && y < 70.0) {
        scan = scans[14];
    }
    else if (y >= 70.1 && y < 85.0) {
        scan = scans[15];
    }
    else if (y >= 85.1 && y < 97.0) {
        scan = scans[16];
    }
    else if (y >= 97.1 && y < 101.2) {
        scan = scans[17];
    }
    else if (y >= 101.3 && y < 105.4) {
        scan = scans[18];
    }
    else if (y >= 105.5 && y < 109.6) {
        scan = scans[19];
    }
    else if (y >= 109.6 && y < 113.8) {
        scan = scans[20];
    }
    else if (y >= 113.9 && y < 118.0) {
        scan = scans[21];
    }
    else if (y >= 118.1 && y < 133.0) {
        scan = scans[22];
    }
    else if (y >= 133.1 && y < 136.6) {
        scan = scans[23];
    }
    else if (y >= 136.7 && y < 140.8) {
        scan = scans[24];
    }
    else if (y >= 140.9 && y < 149.8) {
        scan = scans[25];
    }
    else if (y >= 149.9 && y < 154.0) {
        scan = scans[26];
    }
    else if (y >= 154.1 && y < 158.2) {
        scan = scans[27];
    }
    else if (y >= 158.3 && y < 161.8) {
        scan = scans[28];
    }
    else if (y >= 161.9 && y < 166.0) {
        scan = scans[29];
    }
    
    return scan;
}

/*  Rotates the scan as the sensor is rotate about z-axis   */
void RotateScan() {
    double angle = x / 2;       // Circumference of can is between 700-750 pixels, so approximately 2 pixels is 1 degree of rotation
    
    double bounds[6];
    reader->GetOutput()->GetBounds(bounds);

    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    
    // Compute the center of the image
    double center[3];
    center[0] = (bounds[1] + bounds[0]) / 2.0;
    center[1] = (bounds[3] + bounds[2]) / 2.0;
    center[2] = (bounds[5] + bounds[4]) / 2.0;
    //center[0] = reader->GetWidth() / 2.0;
    //center[1] = reader->GetHeight() / 2.0;
    //center[2] = 0;
    
    // Rotate about the center
    transform->Translate(center[0], center[1], center[2]);
    transform->RotateWXYZ(angle, 0, 0, 1);
    transform->Translate(-center[0], -center[1], -center[2]);
    
    // Reslice
    vtkSmartPointer<vtkImageReslice> reslice = vtkSmartPointer<vtkImageReslice>::New();
    reslice->SetInputConnection(reader->GetOutputPort());
    reslice->SetResliceTransform(transform);
    reslice->SetInterpolationModeToCubic();
    reslice->SetOutputSpacing(reader->GetOutput()->GetSpacing()[0], reader->GetOutput()->GetSpacing()[1], reader->GetOutput()->GetSpacing()[2]);
    reslice->SetOutputOrigin(reader->GetOutput()->GetOrigin()[0], reader->GetOutput()->GetOrigin()[1], reader->GetOutput()->GetOrigin()[2]);
    reslice->SetOutputExtent(reader->GetOutput()->GetExtent()); // Use a larger extent than the original image's to prevent clipping
    imageActor->GetMapper()->SetInputConnection(reslice->GetOutputPort());
    imageActor->Update();
    
    vtkCT2USSimulation* v = vtkCT2USSimulation::New();
    vtkImageData* imageData = reader->GetOutput();
    v->SetInputData(imageData);
    v->SetTransform(transform);
}

/*  Updates the position of the sensor by obtaining serial data
    and changing the changing the CT scan that is displayed.    */
void UpdatePosition(void* arguments)
{
    GetSerialData();
    std::string scan = GetNewScan();
    
    reader->SetFileName(scan.c_str());
    reader->Update();
    
    RotateScan();
}

int main(int, char *[])
{
    // Load scans into array
    std::string directory = getcwd(NULL, 0);
    for (int i=0; i<30; i++) {
        scans[i] = directory + "/scans/" + std::to_string(i+1) + "_Fixed.mhd";
    }
    
    reader = vtkSmartPointer<vtkMetaImageReader>::New();
    reader->SetFileName(scans[0].c_str());      // Choose initial CT scan
    reader->Update();
    
    vtkSmartPointer<vtkProgrammableFilter> programmableFilter = vtkSmartPointer<vtkProgrammableFilter>::New();
    programmableFilter->SetInputConnection(reader->GetOutputPort());
    programmableFilter->SetExecuteMethod(UpdatePosition, programmableFilter);
    
    // Create a mapper and actors
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(programmableFilter->GetOutputPort());
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    
    imageActor = vtkSmartPointer<vtkImageActor>::New();
    imageActor->GetMapper()->SetInputConnection(reader->GetOutputPort());

    // Create a renderer, render window, and interactor
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);
    
    // Initialize must be called prior to creating timer events.
    renderWindowInteractor->Initialize();
    renderWindowInteractor->CreateRepeatingTimer(1);    // Increase or decrease value to change the frequency the position is updated
    
    vtkSmartPointer<vtkCallbackCommand> timerCallback = vtkSmartPointer<vtkCallbackCommand>::New();
    timerCallback->SetCallback ( TimerCallbackFunction );
    timerCallback->SetClientData(programmableFilter);
    
    renderWindowInteractor->AddObserver ( vtkCommand::TimerEvent, timerCallback );
    
    // Add the actors to the scene
    renderer->AddActor(actor);
    renderer->AddActor(imageActor);
    
    // Render and interact
    renderWindow->Render();
    renderWindowInteractor->Start();
    
    return EXIT_SUCCESS;
}

/*  Timer Callback used to update the scene at specified interval lenghts   */
void TimerCallbackFunction ( vtkObject* caller, long unsigned int vtkNotUsed(eventId), void* clientData, void* vtkNotUsed(callData) )
{
    vtkSmartPointer<vtkProgrammableFilter> programmableFilter = static_cast<vtkProgrammableFilter*>(clientData);
    vtkRenderWindowInteractor *iren = static_cast<vtkRenderWindowInteractor*>(caller);
    programmableFilter->Modified();
    iren->Render();
}











/* THE BELOW METHODS ARE NOT CURRENTLY BEING USED */



/*  Updates the position of the x- and y-coordinates by obtaining serial data
 and changing the position of the point in the window.
 NOTE: This method is not currently being used because a point is no longer being displayed.
 It has been replaced with the UpdatePosition method.    */
void UpdatePoint(void* arguments)
{
    GetSerialData();
    
    vtkProgrammableFilter* programmableFilter = static_cast<vtkProgrammableFilter*>(arguments);
    
    vtkPoints* inPts = programmableFilter->GetPolyDataInput()->GetPoints();
    vtkIdType numPts = inPts->GetNumberOfPoints();
    vtkSmartPointer<vtkPoints> newPts = vtkSmartPointer<vtkPoints>::New();
    newPts->SetNumberOfPoints(numPts);
    
    for(vtkIdType i = 0; i < numPts; i++)
    {
        double p[3];
        inPts->GetPoint(i, p);
        newPts->SetPoint(i, p);
    }
    
    double p0[3];
    inPts->GetPoint(0, p0);
    // x and y coordinates need to be scaled down and assigned to the point object:
    p0[0] = x * 0.01;
    p0[1] = y * 0.01;
    newPts->SetPoint(0, p0);
    
    programmableFilter->GetPolyDataOutput()->CopyStructure(programmableFilter->GetPolyDataInput());
    programmableFilter->GetPolyDataOutput()->SetPoints(newPts);
}

/*  Initializes empty scene with a single point in the center.
    Not currently being used. Call from main function if a point needs to be visualized.    */
void DrawPoint() {
    vtkSmartPointer<vtkNamedColors> colors = vtkSmartPointer<vtkNamedColors>::New();
    
    // Create the geometry of a point (the  coordinate)
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    const float p[3] = {0.0, 0.0, 0.0};
    
    // Create the topology of the point (a vertex)
    vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
    // We need an an array of point id's for InsertNextCell.
    vtkIdType pid[1];
    pid[0] = points->InsertNextPoint(p);
    vertices->InsertNextCell(1,pid);
    
    // Create a polydata object
    vtkSmartPointer<vtkPolyData> point = vtkSmartPointer<vtkPolyData>::New();
    
    // Set the points and vertices we created as the geometry and topology of the polydata
    point->SetPoints(points);
    point->SetVerts(vertices);
    
    
    vtkSmartPointer<vtkProgrammableFilter> programmableFilter = vtkSmartPointer<vtkProgrammableFilter>::New();
    programmableFilter->SetInputData(point);
    
    programmableFilter->SetExecuteMethod(UpdatePoint, programmableFilter);
    
    
    // Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(programmableFilter->GetOutputPort());
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(1.0, 0.0, 0.0);
    actor->GetProperty()->SetPointSize(20);
    
    // Create a renderer, render window, and interactor
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->SetWindowName("Point");
    renderWindow->AddRenderer(renderer);
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);
    
    // Initialize must be called prior to creating timer events.
    renderWindowInteractor->Initialize();
    renderWindowInteractor->CreateRepeatingTimer(1);
    
    vtkSmartPointer<vtkCallbackCommand> timerCallback = vtkSmartPointer<vtkCallbackCommand>::New();
    timerCallback->SetCallback ( TimerCallbackFunction );
    timerCallback->SetClientData(programmableFilter);
    
    renderWindowInteractor->AddObserver ( vtkCommand::TimerEvent, timerCallback );
    
    // Add the actor to the scene
    renderer->AddActor(actor);
    renderer->SetBackground(1,1,1); // Background color white
    
    // Render and interact
    renderWindow->Render();
    renderWindowInteractor->Start();
}
