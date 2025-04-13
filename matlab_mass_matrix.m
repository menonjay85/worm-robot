>> robot = importrobot('worm_5dof.urdf')

robot = 

  rigidBodyTree with properties:

     NumBodies: 5
        Bodies: {[1×1 rigidBody]  [1×1 rigidBody]  [1×1 rigidBody]  [1×1 rigidBody]  [1×1 rigidBody]}
          Base: [1×1 rigidBody]
     BodyNames: {'link_1'  'link_2'  'link_3'  'link_4'  'link_5'}
      BaseName: 'base_link'
       Gravity: [0 0 0]
    DataFormat: 'struct'

>> show(robot)

ans = 

  Axes (Primary) with properties:

             XLim: [-0.5000 0.5000]
             YLim: [-0.5000 0.5000]
           XScale: 'linear'
           YScale: 'linear'
    GridLineStyle: '-'
         Position: [0.1300 0.1100 0.7750 0.8150]
            Units: 'normalized'

  Show all properties

Unable to display properties for variable ans because it refers to a deleted object.
>> show(robot)

ans = 

  Axes (Primary) with properties:

             XLim: [-0.5000 0.5000]
             YLim: [-0.5000 0.5000]
           XScale: 'linear'
           YScale: 'linear'
    GridLineStyle: '-'
         Position: [0.1300 0.1100 0.7750 0.8150]
            Units: 'normalized'

  Show all properties

                        ALim: [0 1]
                    ALimMode: 'auto'
                  AlphaScale: 'linear'
                    Alphamap: [0 0.0159 0.0317 0.0476 0.0635 0.0794 0.0952 0.1111 0.1270 … ] (1×64 double)
           AmbientLightColor: [1 1 1]
                BeingDeleted: off
                         Box: off
                    BoxStyle: 'back'
                  BusyAction: 'queue'
               ButtonDownFcn: ''
                        CLim: [0 1]
                    CLimMode: 'auto'
              CameraPosition: [6.0641 6.0641 1.2053]
          CameraPositionMode: 'auto'
                CameraTarget: [0 0 0]
            CameraTargetMode: 'auto'
              CameraUpVector: [0 0 1]
          CameraUpVectorMode: 'auto'
             CameraViewAngle: 8
         CameraViewAngleMode: 'manual'
                    Children: [18×1 Graphics]
                    Clipping: on
               ClippingStyle: '3dbox'
                       Color: [1 1 1]
                  ColorOrder: [7×3 double]
             ColorOrderIndex: 6
                  ColorScale: 'linear'
                    Colormap: [256×3 double]
                 ContextMenu: [0×0 GraphicsPlaceholder]
                   CreateFcn: ''
                CurrentPoint: [2×3 double]
             DataAspectRatio: [1 1 1]
         DataAspectRatioMode: 'manual'
                   DeleteFcn: @robotics.manip.internal.RigidBodyTreeVisualizationHelper.clearCornerAxes
                   FontAngle: 'normal'
                    FontName: 'Helvetica'
                    FontSize: 10
                FontSizeMode: 'auto'
               FontSmoothing: on
                   FontUnits: 'points'
                  FontWeight: 'normal'
                   GridAlpha: 0.1500
               GridAlphaMode: 'auto'
                   GridColor: [0.1500 0.1500 0.1500]
               GridColorMode: 'auto'
               GridLineStyle: '-'
               GridLineWidth: 0.5000
           GridLineWidthMode: 'auto'
            HandleVisibility: 'on'
                     HitTest: on
               InnerPosition: [0.1300 0.1100 0.7750 0.8150]
          InteractionOptions: [1×1 matlab.graphics.interaction.interactionoptions.InteractionOptions]
                Interactions: [1×1 matlab.graphics.interaction.interface.DefaultAxesInteractionSet]
               Interruptible: on
     LabelFontSizeMultiplier: 1.1000
                       Layer: 'bottom'
                      Layout: [0×0 matlab.ui.layout.LayoutOptions]
                      Legend: [0×0 GraphicsPlaceholder]
      LineStyleCyclingMethod: 'aftercolor'
              LineStyleOrder: '-'
         LineStyleOrderIndex: 1
                   LineWidth: 0.5000
              MinorGridAlpha: 0.2500
          MinorGridAlphaMode: 'auto'
              MinorGridColor: [0.1000 0.1000 0.1000]
          MinorGridColorMode: 'auto'
          MinorGridLineStyle: ':'
          MinorGridLineWidth: 0.5000
      MinorGridLineWidthMode: 'auto'
                    NextPlot: 'replace'
             NextSeriesIndex: 6
               OuterPosition: [0 0 1 1]
                      Parent: [1×1 Figure]
               PickableParts: 'visible'
          PlotBoxAspectRatio: [1 1 1]
      PlotBoxAspectRatioMode: 'manual'
                    Position: [0.1300 0.1100 0.7750 0.8150]
          PositionConstraint: 'outerposition'
                  Projection: 'perspective'
                    Selected: off
          SelectionHighlight: on
                  SortMethod: 'depth'
                    Subtitle: [1×1 Text]
          SubtitleFontWeight: 'normal'
                         Tag: 'Primary'
                     TickDir: 'out'
                 TickDirMode: 'auto'
        TickLabelInterpreter: 'tex'
                  TickLength: [0.0100 0.0250]
                  TightInset: [0 0 0 0]
                       Title: [1×1 Text]
     TitleFontSizeMultiplier: 1.1000
             TitleFontWeight: 'bold'
    TitleHorizontalAlignment: 'center'
                     Toolbar: [1×1 AxesToolbar]
                        Type: 'axes'
                       Units: 'normalized'
                    UserData: []
                        View: [135 8]
                     Visible: on
                       XAxis: [1×1 NumericRuler]
               XAxisLocation: 'bottom'
                      XColor: [0.1500 0.1500 0.1500]
                  XColorMode: 'auto'
                        XDir: 'normal'
                       XGrid: on
                      XLabel: [1×1 Text]
                        XLim: [-0.5000 0.5000]
                    XLimMode: 'manual'
                XLimitMethod: 'tickaligned'
                  XMinorGrid: off
                  XMinorTick: off
                      XScale: 'linear'
                       XTick: [-0.5000 0 0.5000]
                  XTickLabel: {3×1 cell}
              XTickLabelMode: 'auto'
          XTickLabelRotation: 0
      XTickLabelRotationMode: 'auto'
                   XTickMode: 'auto'
                       YAxis: [1×1 NumericRuler]
               YAxisLocation: 'left'
                      YColor: [0.1500 0.1500 0.1500]
                  YColorMode: 'auto'
                        YDir: 'normal'
                       YGrid: on
                      YLabel: [1×1 Text]
                        YLim: [-0.5000 0.5000]
                    YLimMode: 'manual'
                YLimitMethod: 'tickaligned'
                  YMinorGrid: off
                  YMinorTick: off
                      YScale: 'linear'
                       YTick: [-0.5000 0 0.5000]
                  YTickLabel: {3×1 cell}
              YTickLabelMode: 'auto'
          YTickLabelRotation: 0
      YTickLabelRotationMode: 'auto'
                   YTickMode: 'auto'
                       ZAxis: [1×1 NumericRuler]
                      ZColor: [0.1500 0.1500 0.1500]
                  ZColorMode: 'auto'
                        ZDir: 'normal'
                       ZGrid: on
                      ZLabel: [1×1 Text]
                        ZLim: [-0.5000 0.5000]
                    ZLimMode: 'manual'
                ZLimitMethod: 'tickaligned'
                  ZMinorGrid: off
                  ZMinorTick: off
                      ZScale: 'linear'
                       ZTick: [-0.5000 0 0.5000]
                  ZTickLabel: {3×1 cell}
              ZTickLabelMode: 'auto'
          ZTickLabelRotation: 0
      ZTickLabelRotationMode: 'auto'
                   ZTickMode: 'auto'

>> massMatrix(robot)
Error using robotics.manip.internal.error
The DataFormat property of rigidBodyTree object is currently set as 'struct'. To use RigidBodyTree dynamics
functions, DataFormat property must be set to either 'column' or 'row'.

Error in robotics.manip.internal.RigidBodyTree/validateDynamicsFunctionInputs (line 1510)
                robotics.manip.internal.error('rigidbodytree:DynamicsFunctionsUseVectorsOnly');

Error in rigidBodyTree/massMatrix (line 920)
            q = validateDynamicsFunctionInputs(obj.TreeInternal, false, varargin{:});
 
>> q = randomConfiguration(robot);
>> H = massMatrix(atlas,q)
Unrecognized function or variable 'atlas'.
 
>> H = massMatrix(robot,q)
Error using robotics.manip.internal.error
The DataFormat property of rigidBodyTree object is currently set as 'struct'. To use RigidBodyTree dynamics
functions, DataFormat property must be set to either 'column' or 'row'.

Error in robotics.manip.internal.RigidBodyTree/validateDynamicsFunctionInputs (line 1510)
                robotics.manip.internal.error('rigidbodytree:DynamicsFunctionsUseVectorsOnly');

Error in rigidBodyTree/massMatrix (line 920)
            q = validateDynamicsFunctionInputs(obj.TreeInternal, false, varargin{:});
 
>> robot = importrobot('worm_5dof.urdf');
>> robot.DataFormat = 'row';
>> 
>> q = randomConfiguration(robot);  
>> H = massMatrix(robot, q);
>> show(robot,q);
>> clear al
>> robot = importrobot('worm_5dof.urdf',DataFormat="row", Gravity=[0 0 -9.81])
Error using importrobot/processInputs
'Gravity' is not a recognized parameter. For a list of valid name-value pair arguments, see the documentation
for this function.

Error in importrobot (line 159)
        [parsedInput, inputType] = processInputs(input, varargin{:});
 
>> robot = importrobot('worm_5dof.urdf',DataFormat="row")

robot = 

  rigidBodyTree with properties:

     NumBodies: 5
        Bodies: {[1×1 rigidBody]  [1×1 rigidBody]  [1×1 rigidBody]  [1×1 rigidBody]  [1×1 rigidBody]}
          Base: [1×1 rigidBody]
     BodyNames: {'link_1'  'link_2'  'link_3'  'link_4'  'link_5'}
      BaseName: 'base_link'
       Gravity: [0 0 0]
    DataFormat: 'row'

>> q = randomConfiguration(robot);
>> H = massMatrix(robot, q);
>> show(robot,q);
>> robot = importrobot('worm_5dof.urdf',DataFormat="row")

robot = 

  rigidBodyTree with properties:

     NumBodies: 5
        Bodies: {[1×1 rigidBody]  [1×1 rigidBody]  [1×1 rigidBody]  [1×1 rigidBody]  [1×1 rigidBody]}
          Base: [1×1 rigidBody]
     BodyNames: {'link_1'  'link_2'  'link_3'  'link_4'  'link_5'}
      BaseName: 'base_link'
       Gravity: [0 0 0]
    DataFormat: 'row'

>> H = massMatrix(robot)

H =

    0.0112    0.0076    0.0044    0.0018    0.0002
    0.0076    0.0054    0.0032    0.0014    0.0002
    0.0044    0.0032    0.0021    0.0009    0.0001
    0.0018    0.0014    0.0009    0.0005    0.0001
    0.0002    0.0002    0.0001    0.0001    0.0000

>> 