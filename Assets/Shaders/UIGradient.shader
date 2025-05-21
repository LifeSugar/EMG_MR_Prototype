Shader"Custom/UI/Gradient"
{
    Properties
    {
        [PreRenderData] _MainTex ("MainTexture", 2D) = "white" {}
        _ColorTint("ColorTint", Color) = (1, 1, 1, 1)
        
        //Stencil
        [HideInInspector] _StencilComp		("Stencil Comparison", Float) = 0
	    [HideInInspector] _Stencil			("Stencil ID", Float) = 0
	    [HideInInspector] _StencilOp		("Stencil Operation", Float) = 0
	    [HideInInspector] _StencilWriteMask	("Stencil Write Mask", Float) = 255
	    [HideInInspector] _StencilReadMask	("Stencil Read Mask", Float) = 255
    	
    	 _ColorMask ("Color Mask", Float) = 15
        [Toggle(UNITY_UI_ALPHACLIP)] _UseUIAlphaClip ("Use Alpha Clip", Float) = 0

        _LinearGradientColor1 ("Linear Gradient Color 1", Color) = (1, 1, 1, 1)
        _LinearGradientColor2 ("Linear Gradient Color 2", Color) = (1, 1, 1, 1)
        _LinearGradientStart ("Linear Gradient Start", Vector) = (0, 0, 0, 0)
        _LinearGradientEnd ("Linear Gradient End", Vector) = (1, 1, 0, 0)
        
    }
    
    SubShader
    {
    	Tags
    	{
    		"Queue" = "Transparent"
    		"IgnoreProjector" = "True"
    		"RenderPipeline" = "UniversalPipeline"
    	}
    	
    	Stencil
	    {
		    Ref [_Stencil]
		    Comp [_StencilComp]
		    Pass [_StencilOp]
		    ReadMask [_StencilReadMask]
		    WriteMask [_StencilWriteMask]
	    }

	    Cull Off
        Lighting Off
        ZWrite Off
        ZTest LEqual
        Blend SrcAlpha OneMinusSrcAlpha, OneMinusDstAlpha One
        ColorMask [_ColorMask]
        
        Pass
        {
        	Name "Default"
        	Tags
        	{
        		"LightMode" = "SRPDefaultUnlit"
        	}
        }
    }
}