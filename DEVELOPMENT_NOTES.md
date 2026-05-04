# 1
cluster_boundary_poly = intersection.geoms[0]   # 硬 编码取第一个!

当聚类凸包跟水域相交是个 MultiPolygon时,geoms[0] 不一定是包含当前核心点的那个，可能选到一个完全无关的子区域作为边界

我改成了：

if cluster_boundary_poly is not None: break cluster_boundary_poly = _pick_nearest_polygon( intersection, current_pt_geom # 找不到就选最近的 

# 2
paths_dict 里只存了 A→B 或 B→A 其中一个方向(两个方向是同一条路径)。入口路径遇到反向 key 的时候会把 path 翻转回来,但出口路径没做这个处理。结果出口路径闭环出口路径的"靠水域端"(path[0] 还是 path[-1])随机有情况是反的。

# 3
把 water_poly 外界 + cluster_boundary_poly 外界合并成一个 LineString

这个合并求交的时候入口点可能落在 water_poly 外边界上，这个点有可能在聚类范围外。因为两条边界被 union 成一个几何体后,距离更近的那个边界胜出,但"距离近"不一定是"在聚类里"。我是加了一个判断

# 4
if not points: return None #  没交点就放弃

路径端点有时候就停在水域内部 1-2 米的位置。这种情况 buffer(0.1) 不行。原来直接返回 None,于是  得手动补救。我是加了一个，端点投影

————————

但是改完134之后还是有问题，改完2之后才好的，有可能就只有2是问题