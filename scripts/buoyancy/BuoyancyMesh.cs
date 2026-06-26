using Godot;
using System;
using System.Collections.Generic;

/// <summary>
/// 基于网格几何的浮力 / 水动力学脚本（C# 重构版）
/// ============================================================
/// 来源：参考 https://github.com/q-qo-o/godot-floating-objects 的 buoyancy_mesh.gd
/// 算法原理：
///
///   1. 在 _Ready() 时遍历所有子 MeshInstance3D，提取其三角形
///      顶点（局部坐标），并缓存每个 mesh 相对刚体的变换。
///
///   2. 每物理帧（_IntegrateForces）:
///      a. 将三角形顶点变换到世界坐标系。
///      b. 用水面平面 y = water_height 对每个三角形做"水线裁剪"，
///         得到完全位于水面以下的子三角形集合。
///      c. 对每个水下三角形与水面参考点构造四面体，用
///           V = (1/6) * r0 · (r1 × r2)
///         计算其有向体积，按体积权重累加形心位置。
///      d. 在累加得到的浮心处施加 F = ρ·g·V_sub。
///
///   3. 可选：在刚体局部系下计算线性 + 二次阻尼力 / 力矩
///      （Fossen 水下航行器阻尼模型）。
///
/// 相对原版增加的旋钮：
///   - BuoyancyMultiplier：浮力倍率（默认 1.0 = 纯阿基米德浮力）。
///     折纸机器人是薄壳结构，其实际排开水的体积（外廓包围体积）远大于
///     网格实体体积，因此乘一个 &gt;1 的倍率在物理上是合理的，可让机器人
///     带正浮力浮在水面。
///
/// 使用要求：
///   - 节点本身必须是 RigidBody3D。
///   - 必须有至少一个子 MeshInstance3D 提供几何信息。
///   - 网格应当是封闭的且法向朝外（基础形状和大多数美术资产满足）。
///
/// 场景结构示例：
///   FloatingBox (RigidBody3D, 挂载本脚本)
///   ├── CollisionShape3D       # 物理碰撞
///   └── MeshInstance3D         # 浮力几何
/// </summary>
public partial class BuoyancyMesh : RigidBody3D
{
	// ============================================================
	// 导出参数
	// ============================================================

	/// <summary>水面节点。其全局 Y 坐标作为水面高度（支持移动水面）。
	/// 若为 null，则在 _Ready 时尝试查找名为 "WaterSurface" 的节点；
	/// 若仍找不到，水面高度回退到 FallbackWaterLevel（默认 0.0）。</summary>
	[Export] public Node3D WaterSurfaceNode;

	private const float FallbackWaterLevel = 0.0f;

	/// <summary>流体密度，水为 1000 kg/m³</summary>
	[Export] public float FluidDensity = 1000.0f;

	/// <summary>浮力倍率（默认 2.0，折纸薄壳需要 &gt;1 才能正浮力）。
	/// 注意：若编辑器未 Build Project，.tscn 中的值不会加载到此字段，
	/// 因此默认值必须与场景文件保持一致。</summary>
	[Export] public float BuoyancyMultiplier = 2.0f;

	/// <summary>基础线性阻尼（按浸没比例缩放）</summary>
	[Export] public float WaterDrag = 1.5f;

	/// <summary>基础角阻尼（按浸没比例缩放）</summary>
	[Export] public float WaterAngularDrag = 1.5f;

	[ExportGroup("水动力学阻尼 (Body Frame)")]
	/// <summary>线性阻尼系数（刚体局部系）— Forward / Lateral / Vertical</summary>
	[Export] public Vector3 LinearDampingTranslational = Vector3.Zero;

	/// <summary>角速度线性阻尼系数（刚体局部系）— Roll / Pitch / Yaw</summary>
	[Export] public Vector3 LinearDampingRotational = Vector3.Zero;

	/// <summary>平移二次阻尼系数（形状阻力）</summary>
	[Export] public Vector3 QuadraticDampingTranslational = Vector3.Zero;

	/// <summary>转动二次阻尼系数</summary>
	[Export] public Vector3 QuadraticDampingRotational = Vector3.Zero;

	/// <summary>平移附加质量系数</summary>
	[Export] public Vector3 AddedMassTranslational = Vector3.Zero;

	/// <summary>转动附加质量系数（附加惯性）</summary>
	[Export] public Vector3 AddedMassRotational = Vector3.Zero;

	[ExportGroup("调试")]
	/// <summary>是否在调试器中暴露内部状态</summary>
	[Export] public bool DebugDraw = false;

	// ============================================================
	// 内部状态
	// ============================================================

	/// <summary>重力加速度（从 ProjectSettings 读取）</summary>
	private float _gravity;

	/// <summary>当前浸没比例 [0, 1]，供外部脚本读取</summary>
	public float SubmergedRatio = 0.0f;

	/// <summary>当前垂直速度，供外部读取</summary>
	public float VerticalVelocity = 0.0f;

	/// <summary>上一帧局部坐标系下的线速度</summary>
	private Vector3 _prevLinVelBody = Vector3.Zero;

	/// <summary>上一帧局部坐标系下的角速度</summary>
	private Vector3 _prevAngVelBody = Vector3.Zero;

	/// <summary>缓存的网格三角形数据</summary>
	private readonly List<MeshTriangleData> _meshTriangles = new();

	/// <summary>所有 mesh 的总体积（用于 submerged_ratio 归一化）</summary>
	private float _totalMeshVolume = 0.0f;

	/// <summary>最近一帧的浮心（世界坐标）</summary>
	[Export] public Vector3 LastBuoyancyCenterWorld;

	/// <summary>最近一帧的浮力大小</summary>
	[Export] public float LastTotalForce = 0.0f;

	/// <summary>调试帧计数器（每60帧打印一次日志）</summary>
	private int _debugFrameCounter = 0;

	/// <summary>单个 mesh 的三角形缓存数据</summary>
	private struct MeshTriangleData
	{
		public Vector3[] Tris;      // 每 3 个连续顶点构成一个三角形（局部坐标）
		public Transform3D Xform;   // mesh 相对刚体的变换
		public Node3D Owner;        // 来源 MeshInstance3D
	}

	/// <summary>裁剪后的三角形</summary>
	private struct Triangle
	{
		public Vector3 V0;
		public Vector3 V1;
		public Vector3 V2;
	}

	// ============================================================
	// 初始化
	// ============================================================

	public override void _Ready()
	{
		base._Ready();

		// 加入 floating_bodies 组，便于 water_surface.gd 等脚本检索
		AddToGroup("floating_bodies");
		_gravity = (float)ProjectSettings.GetSetting("physics/3d/default_gravity");

		// 若未指定水面节点，尝试从当前场景查找名为 "WaterSurface" 的节点
		if (WaterSurfaceNode == null)
		{
			var current = GetTree().CurrentScene;
			if (current != null)
			{
				WaterSurfaceNode = current.GetNodeOrNull<Node3D>("WaterSurface");
			}
		}
		if (WaterSurfaceNode == null)
		{
			GD.PushWarning($"BuoyancyMesh: 未找到水面节点，将使用回退水面高度 {FallbackWaterLevel}");
		}

		CollectMeshes();

		// 初始化附加质量历史状态
		_prevLinVelBody = Vector3.Zero;
		_prevAngVelBody = Vector3.Zero;
	}

	/// <summary>收集所有子 MeshInstance3D 的三角形，并计算总体积</summary>
	private void CollectMeshes()
	{
		_meshTriangles.Clear();
		_totalMeshVolume = 0.0f;
		CollectMeshesRecursive(this);

		// 用四面体体积分计算每个 mesh 的总体积
		foreach (var entry in _meshTriangles)
		{
			_totalMeshVolume += ComputeMeshVolume(entry.Tris);
		}

		// 防止除零
		if (_totalMeshVolume < 1e-6f)
		{
			_totalMeshVolume = 1.0f;
		}
	}

	/// <summary>递归遍历子节点，收集 MeshInstance3D</summary>
	private void CollectMeshesRecursive(Node node)
	{
		if (node is MeshInstance3D mi)
		{
			if (mi.Mesh != null)
			{
				var tris = ExtractTrianglesFromMesh(mi.Mesh);
				if (tris.Length >= 3)
				{
					// 计算 mesh 相对刚体（self）的变换
					var xform = GlobalTransform.AffineInverse() * mi.GlobalTransform;
					_meshTriangles.Add(new MeshTriangleData
					{
						Tris = tris,
						Xform = xform,
						Owner = mi
					});
				}
			}
		}

		foreach (var child in node.GetChildren())
		{
			CollectMeshesRecursive(child);
		}
	}

	/// <summary>从 Mesh 资源中提取所有三角形顶点
	/// 返回值：每 3 个连续元素构成一个三角形
	/// 支持带索引和不带索引两种格式</summary>
	private Vector3[] ExtractTrianglesFromMesh(Mesh mesh)
	{
		var result = new List<Vector3>();

		for (int surfaceIdx = 0; surfaceIdx < mesh.GetSurfaceCount(); surfaceIdx++)
		{
			var arrays = mesh.SurfaceGetArrays(surfaceIdx);
			if (arrays.Count == 0)
				continue;

			var vertsVariant = arrays[(int)Mesh.ArrayType.Vertex];
			if (vertsVariant.VariantType == Variant.Type.Nil)
				continue;

			var verts = (Vector3[])vertsVariant;

			var indicesVariant = arrays[(int)Mesh.ArrayType.Index];
			int[] indices;
			if (indicesVariant.VariantType != Variant.Type.Nil)
			{
				indices = (int[])indicesVariant;
			}
			else
			{
				indices = Array.Empty<int>();
			}

			if (indices.Length > 0)
			{
				for (int i = 0; i < indices.Length; i += 3)
				{
					if (i + 2 < indices.Length)
					{
						result.Add(verts[indices[i]]);
						result.Add(verts[indices[i + 1]]);
						result.Add(verts[indices[i + 2]]);
					}
				}
			}
			else
			{
				for (int i = 0; i < verts.Length; i += 3)
				{
					if (i + 2 < verts.Length)
					{
						result.Add(verts[i]);
						result.Add(verts[i + 1]);
						result.Add(verts[i + 2]);
					}
				}
			}
		}

		return result.ToArray();
	}

	/// <summary>计算封闭网格的总体积
	/// 原理：对每个三角形 (v0, v1, v2) 与原点构成四面体，
	///       有向体积 V = (1/6) v0 · (v1 × v2)
	/// 要求：网格封闭且法向朝外。</summary>
	private float ComputeMeshVolume(Vector3[] tris)
	{
		float vol = 0.0f;
		int i = 0;
		while (i + 2 < tris.Length)
		{
			Vector3 v0 = tris[i];
			Vector3 v1 = tris[i + 1];
			Vector3 v2 = tris[i + 2];
			vol += v0.Dot(v1.Cross(v2)) / 6.0f;
			i += 3;
		}
		return Mathf.Abs(vol);
	}

	// ============================================================
	// 物理积分（每物理帧调用）
	// ============================================================

	public override void _IntegrateForces(PhysicsDirectBodyState3D state)
	{
		if (_meshTriangles.Count == 0)
			return;

		float waterHeight = GetWaterHeight();

		// 累加器
		float totalSubmergedVolume = 0.0f;
		Vector3 weightedCentroid = Vector3.Zero;

		var bodyXform = state.Transform;

		foreach (var entry in _meshTriangles)
		{
			Vector3[] localTris = entry.Tris;
			Transform3D meshXform = entry.Xform;
			Transform3D worldXform = bodyXform * meshXform;

			int i = 0;
			while (i + 2 < localTris.Length)
			{
				Vector3 v0w = worldXform * localTris[i];
				Vector3 v1w = worldXform * localTris[i + 1];
				Vector3 v2w = worldXform * localTris[i + 2];

				var clipped = ClipTriangleToWater(v0w, v1w, v2w, waterHeight);

				foreach (var tri in clipped)
				{
					Vector3 t0 = tri.V0;
					Vector3 t1 = tri.V1;
					Vector3 t2 = tri.V2;

					// 用水面上一个参考点与三角形构造四面体
					Vector3 reference = new(0.0f, waterHeight, 0.0f);
					Vector3 r0 = t0 - reference;
					Vector3 r1 = t1 - reference;
					Vector3 r2 = t2 - reference;

					// 四面体的有向体积（标量三重积）
					float dv = r0.Dot(r1.Cross(r2)) / 6.0f;

					// 四面体的几何形心（四顶点平均）
					Vector3 dc = (reference + t0 + t1 + t2) / 4.0f;

					totalSubmergedVolume += dv;
					weightedCentroid += dc * dv;
				}

				i += 3;
			}
		}

		// 取绝对值判断，但浮心计算仍用带符号的体积
		float submergedVolume = Mathf.Abs(totalSubmergedVolume);
		_debugFrameCounter++;
		bool shouldLog = _debugFrameCounter % 60 == 0 && DebugDraw;
		if (submergedVolume > 1e-9f)
		{
			// 加权平均得到真实浮心（世界坐标）
			Vector3 centroidWorld = weightedCentroid / totalSubmergedVolume;
			SubmergedRatio = Mathf.Clamp(submergedVolume / _totalMeshVolume, 0.0f, 1.0f);

			// 阿基米德浮力：F = ρ · g · V，方向竖直向上
			Vector3 buoyancyForce = Vector3.Up * submergedVolume * FluidDensity * _gravity * BuoyancyMultiplier;
			LastTotalForce = buoyancyForce.Length();
			LastBuoyancyCenterWorld = centroidWorld;

			// state.ApplyForce(force, position) 的 position 参数是世界坐标
			state.ApplyForce(buoyancyForce, centroidWorld);

			if (shouldLog)
				GD.Print($"[Buoyancy] {Name}: V={submergedVolume:E2}, F={LastTotalForce:F3}, ratio={SubmergedRatio:F2}, mult={BuoyancyMultiplier}, g={_gravity}");

			// 基础阻尼
			state.LinearVelocity *= 1.0f - WaterDrag * state.Step * SubmergedRatio;
			state.AngularVelocity *= 1.0f - WaterAngularDrag * state.Step * SubmergedRatio;

			// 可选的物理准确阻尼模型
			ApplyHydrodynamicDamping(state);
		}
		else
		{
			SubmergedRatio = 0.0f;
			LastTotalForce = 0.0f;
			if (shouldLog)
				GD.Print($"[Buoyancy] {Name}: no submersion (vol={submergedVolume:E2}), water_h={waterHeight:F3}");
		}

		// 更新公开状态供外部读取
		VerticalVelocity = state.LinearVelocity.Y;
		SetMeta("submerged_ratio", SubmergedRatio);
		SetMeta("vertical_velocity", VerticalVelocity);
	}

	// ============================================================
	// 水动力学阻尼（Fossen 模型）
	// ============================================================

	/// <summary>在刚体局部坐标系下计算线性 + 二次阻尼，再变换回世界系施加。</summary>
	private void ApplyHydrodynamicDamping(PhysicsDirectBodyState3D state)
	{
		// 几乎不浸没时跳过
		if (SubmergedRatio <= 0.001f)
		{
			_prevLinVelBody = Vector3.Zero;
			_prevAngVelBody = Vector3.Zero;
			return;
		}

		Transform3D bodyXform = state.Transform;
		Basis rot = bodyXform.Basis;

		// 将世界系速度变换到刚体局部系
		Vector3 linVelBody = rot.Transposed() * state.LinearVelocity;
		Vector3 angVelBody = rot.Transposed() * state.AngularVelocity;

		// 1. 线性阻尼 F = -D · v
		Vector3 fLin = -LinearDampingTranslational * linVelBody;
		Vector3 tLin = -LinearDampingRotational * angVelBody;

		// 2. 二次阻尼 F = -D · |v| · v
		Vector3 fQuad = -QuadraticDampingTranslational * new Vector3(
			Mathf.Abs(linVelBody.X) * linVelBody.X,
			Mathf.Abs(linVelBody.Y) * linVelBody.Y,
			Mathf.Abs(linVelBody.Z) * linVelBody.Z
		);
		Vector3 tQuad = -QuadraticDampingRotational * new Vector3(
			Mathf.Abs(angVelBody.X) * angVelBody.X,
			Mathf.Abs(angVelBody.Y) * angVelBody.Y,
			Mathf.Abs(angVelBody.Z) * angVelBody.Z
		);

		// 3. 附加质量 F_am = -M · a
		float dt = Mathf.Max(state.Step, 1e-6f);
		Vector3 linAccBody = (linVelBody - _prevLinVelBody) / dt;
		Vector3 angAccBody = (angVelBody - _prevAngVelBody) / dt;

		Vector3 fAm = -AddedMassTranslational * linAccBody;
		Vector3 tAm = -AddedMassRotational * angAccBody;

		// 汇总并施加
		Vector3 totalForceBody = (fLin + fQuad + fAm) * SubmergedRatio;
		Vector3 totalTorqueBody = (tLin + tQuad + tAm) * SubmergedRatio;

		state.ApplyCentralForce(rot * totalForceBody);
		state.ApplyTorque(rot * totalTorqueBody);

		// 保存当前速度用于下一帧
		_prevLinVelBody = linVelBody;
		_prevAngVelBody = angVelBody;
	}

	// ============================================================
	// 三角形 - 水面裁剪
	// ============================================================

	/// <summary>将世界坐标系下的三角形按水平面 y = h 裁剪，
	/// 返回水面以下部分（y &lt; h）的三角形列表。</summary>
	private List<Triangle> ClipTriangleToWater(Vector3 v0, Vector3 v1, Vector3 v2, float h)
	{
		Vector3[] pts = { v0, v1, v2 };
		bool[] below = { v0.Y < h, v1.Y < h, v2.Y < h };

		int count = 0;
		foreach (var b in below)
		{
			if (b) count++;
		}

		if (count == 0)
			return new List<Triangle>();
		if (count == 3)
			return new List<Triangle> { new() { V0 = v0, V1 = v1, V2 = v2 } };

		// 求线段与平面 y = h 的交点
		Vector3 Intersect(Vector3 a, Vector3 b)
		{
			float dy = b.Y - a.Y;
			if (Mathf.Abs(dy) < 1e-9f)
				return a;
			float t = (h - a.Y) / dy;
			return a + (b - a) * t;
		}

		if (count == 1)
		{
			int i = Array.IndexOf(below, true);
			Vector3 p0 = pts[i];
			Vector3 p1 = pts[(i + 1) % 3];
			Vector3 p2 = pts[(i + 2) % 3];
			return new List<Triangle>
			{
				new() { V0 = p0, V1 = Intersect(p0, p1), V2 = Intersect(p0, p2) }
			};
		}

		// count == 2
		int j = Array.IndexOf(below, false);
		Vector3 qAbove = pts[j];
		Vector3 q1 = pts[(j + 1) % 3];
		Vector3 q2 = pts[(j + 2) % 3];
		Vector3 i1a = Intersect(q1, qAbove);
		Vector3 i2a = Intersect(q2, qAbove);
		return new List<Triangle>
		{
			new() { V0 = q1, V1 = q2, V2 = i2a },
			new() { V0 = q1, V1 = i2a, V2 = i1a }
		};
	}

	// ============================================================
	// 工具函数
	// ============================================================

	/// <summary>获取当前水面高度</summary>
	private float GetWaterHeight()
	{
		if (WaterSurfaceNode != null)
			return WaterSurfaceNode.GlobalPosition.Y;
		return FallbackWaterLevel;
	}

	/// <summary>公开接口：若运行时改变了子 MeshInstance3D 或 Mesh 资源，
	/// 可调用此方法刷新缓存</summary>
	public void RefreshMeshes()
	{
		CollectMeshes();
		_prevLinVelBody = Vector3.Zero;
		_prevAngVelBody = Vector3.Zero;
	}
}
