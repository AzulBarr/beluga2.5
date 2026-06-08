import cv2
import numpy as np
import matplotlib.pyplot as plt
import argparse


def load_map(path):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError(f"No se pudo cargar {path}")
    return img

def align_maps(ref, target):
    # Convertir a float32 para ECC
    ref_f = ref.astype(np.float32) / 255.0
    tgt_f = target.astype(np.float32) / 255.0

    warp = np.eye(2, 3, dtype=np.float32)

    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 5000, 1e-6)

    try:
        cc, warp = cv2.findTransformECC(
            ref_f,
            tgt_f,
            warp,
            cv2.MOTION_EUCLIDEAN,
            criteria
        )
    except cv2.error as e:
        print("⚠️ ECC falló, usando sin alinear")
        return target, warp

    aligned = cv2.warpAffine(
        target,
        warp,
        (ref.shape[1], ref.shape[0]),
        flags=cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=205   # gris = desconocido
    )

    return aligned, warp

def overlay_maps_occ(ref, aligned):
    # ref en rojo, aligned en verde
    ref_occ = (ref < 100)
    aligned_occ = (aligned < 100)

    overlay = np.zeros((ref.shape[0], ref.shape[1], 3), dtype=np.uint8)
    overlay[:, :, 0] = ref_occ * 255
    overlay[:, :, 1] = aligned_occ * 255
    return overlay

def overlay_maps_free(ref, aligned):
    # ref en rojo, aligned en verde
    ref_occ = (ref > 250)
    aligned_occ = (aligned > 250)

    overlay = np.zeros((ref.shape[0], ref.shape[1], 3), dtype=np.uint8)
    overlay[:, :, 0] = ref_occ * 255
    overlay[:, :, 1] = aligned_occ * 255
    return overlay

def compute_iou(ref, aligned, unknown_val=205, occ_thresh=100):
    # 1. máscaras de válido (no unknown)
    valid_ref = ref != unknown_val
    valid_aligned = aligned != unknown_val
    valid = valid_ref & valid_aligned

    # 2. máscaras de ocupación
    ref_occ = ref < occ_thresh
    aligned_occ = aligned < occ_thresh

    # 3. aplicar máscara válida
    ref_occ = ref_occ & valid
    aligned_occ = aligned_occ & valid

    # 4. intersección y unión
    intersection = np.logical_and(ref_occ, aligned_occ)
    union = np.logical_or(ref_occ, aligned_occ)

    # 5. evitar división por 0
    if np.sum(union) == 0:
        return 0.0

    iou = np.sum(intersection) / np.sum(union)
    return iou

def crop_unknown_borders(img, unknown_val=205):
    # máscara de píxeles que NO son unknown
    mask = img != unknown_val

    coords = np.argwhere(mask)

    if coords.size == 0:
        return img  # mapa vacío

    y_min, x_min = coords.min(axis=0)
    y_max, x_max = coords.max(axis=0)

    cropped = img[y_min:y_max+1, x_min:x_max+1]

    return cropped

def classify_map(img, occ_thresh=100, free_thresh=250):
    occ = img < occ_thresh
    free = img > free_thresh
    unknown = ~(occ | free)
    return occ, free, unknown

def confusion_matrix_map(ref, aligned, unknown_val=205):
    ref_occ, ref_free, ref_unk = classify_map(ref)
    est_occ, est_free, est_unk = classify_map(aligned)

    # máscara válida (excluir unknown en cualquiera)
    valid = ~(ref_unk | est_unk)

    TP = np.sum((ref_occ & est_occ) & valid)
    TN = np.sum((ref_free & est_free) & valid)
    FP = np.sum((ref_free & est_occ) & valid)
    FN = np.sum((ref_occ & est_free) & valid)

    return TP, TN, FP, FN

def compute_metrics(TP, TN, FP, FN):
    accuracy = (TP + TN) / (TP + TN + FP + FN)

    precision = TP / (TP + FP) if (TP + FP) > 0 else 0
    recall = TP / (TP + FN) if (TP + FN) > 0 else 0

    f1 = (2 * precision * recall / (precision + recall)
          if (precision + recall) > 0 else 0)

    return accuracy, precision, recall, f1

def plot_confusion_matrix(TP, TN, FP, FN):
    cm = np.array([
        [TP, FN],
        [FP, TN]
    ])

    plt.figure(figsize=(4,4))
    plt.imshow(cm, cmap='Blues')

    labels = [["TP", "FN"], ["FP", "TN"]]

    for i in range(2):
        for j in range(2):
            plt.text(j, i, f"{labels[i][j]}\n{cm[i,j]}",
                     ha='center', va='center', color='black')

    plt.xticks([0,1], ["Pred Occ", "Pred Free"])
    plt.yticks([0,1], ["True Occ", "True Free"])

    plt.title("Confusion Matrix (pixels)")
    plt.colorbar()
    plt.tight_layout()
    plt.show()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map1", help="Mapa referencia (.pgm)")
    parser.add_argument("map2", help="Mapa a alinear (.pgm)")
    args = parser.parse_args()

    map1 = load_map(args.map1)
    map2 = load_map(args.map2)

    map1_bin = crop_unknown_borders(map1)
    map2_bin = map2

    aligned, warp = align_maps(map1_bin, map2_bin)

    overlay_occ = overlay_maps_occ(map1_bin, aligned)

    overlay_free = overlay_maps_free(map1_bin, aligned)

    iou = compute_iou(map1_bin, aligned)
    print(f"IoU (sin unknown): {iou:.4f}")

    print("Warp matrix:")
    print(warp)

    plt.figure(figsize=(15,5))

    plt.subplot(2,2,1)
    plt.title("Mapa referencia")
    plt.imshow(map1_bin, cmap='gray')

    plt.subplot(2,2,2)
    plt.title("Mapa alineado")
    plt.imshow(aligned, cmap='gray')

    plt.subplot(2,2,3)
    plt.title("Overlay obstacles (rojo=ref, verde=estimado)")
    plt.imshow(overlay_occ)

    plt.subplot(2,2,4)
    plt.title("Overlay free space (rojo=ref, verde=estimado)")
    plt.imshow(overlay_free)

    plt.tight_layout()
    plt.show()

    TP, TN, FP, FN = confusion_matrix_map(map1_bin, aligned)

    print("\nConfusion Matrix:")
    print(f"TP (ocupado bien): {TP}")
    print(f"TN (libre bien):   {TN}")
    print(f"FP (falso ocupado): {FP}")
    print(f"FN (faltan paredes): {FN}")

    accuracy, precision, recall, f1 = compute_metrics(TP, TN, FP, FN)

    print("\nMetrics:")
    print(f"Accuracy:  {accuracy:.4f}")
    print(f"Precision: {precision:.4f}")
    print(f"Recall:    {recall:.4f}")
    print(f"F1-score:  {f1:.4f}")

    plot_confusion_matrix(TP, TN, FP, FN)
    
if __name__ == "__main__":
    main()